package org.firstinspires.ftc.teamcode.bench;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;

import org.firstinspires.ftc.teamcode.calibration.Calibration;

/**
 * PathMaker 1's control law (pathmaker/PathManager.calculateCorrectionPower), ported to run
 * inside Pedro's Follower for the benchmark. The behaviour is kept: x, y and heading are each
 * driven toward their goal with full power far away and a linear ramp near it, all-or-nothing
 * "energy" braking, powers shared by the sum of the three axes, pass-through steps that end when
 * an axis passes its goal, and per-axis start delays.
 *
 * Fixed, as the review recommends (docs/PATHING-PEDRO-VS-PATHMAKER.md, section 2.1):
 *  W1  no blocking 40 ms sleep: the Follower loop runs as fast as it can;
 *  W3  a minimum power (the measured kS) while outside the target zone, so the robot arrives
 *      instead of stalling just short of the goal;
 *  W5  the VELOCITY ramp mode is gone (it divided by a constant 1.0);
 *  W7  a step only ends when the robot is also slow, not just inside the zone.
 * The power-up limit is per second, because the loop is no longer a fixed 40 ms.
 */
public class PathMaker1Algorithm extends PathMakerAlgorithm {

    /** Live-editable in Panels. Defaults are the values PathDetails.initializePath() used. */
    @Configurable
    public static class Settings {
        public static double rampReachX_in = 4, rampReachY_in = 5, rampReachTurn_deg = 30;
        /** PathMaker 1 used 10°; tightened so the heading test means something. */
        public static double targetZoneXY_in = 1, targetZoneTurn_deg = 2;
        public static double minVelocityXY_inPerSec = 2, minVelocityTurn_degPerSec = 4;
        /** Was 0.05 per 40 ms cycle. */
        public static double powerStepUpPerSecond = 1.25;
        public static double energyThreshold = 8000, robotMassKg = 10;
        public static boolean minimumPowerFix = true; // W3
        /** The minimum power is this many times the measured kS, so it reliably moves the robot. */
        public static double minimumPowerOverKS = 1.25;
        public static boolean speedCheckFix = true;   // W7
    }

    public double powerScaling = 1;

    private final double[] last = new double[3];
    private final int[] initialSign = new int[3];

    @Override
    protected void startStep(StepSpec s, MotionState state) {
        double[] e = errors(s, state.pose());
        double[] zone = zones();
        for (int i = 0; i < 3; i++) initialSign[i] = Math.abs(e[i]) > zone[i] ? (int) Math.signum(e[i]) : 0;
    }

    @Override
    protected DrivePowers step(StepSpec s, MotionState state, double elapsedMs, double dt) {
        Pose pose = state.pose();
        Twist tw = state.twist();
        double[] e = errors(s, pose);
        double[] zone = zones();
        double[] reach = {Settings.rampReachX_in, Settings.rampReachY_in, Settings.rampReachTurn_deg};
        double[] delay = {s.delayXMs, s.delayYMs, s.delayHMs};
        double h = pose.heading();
        double kSF = Calibration.getDouble("pathmaker.kSForward"), kSS = Calibration.getDouble("pathmaker.kSStrafe");
        double c = Math.abs(Math.cos(h)), sn = Math.abs(Math.sin(h));
        double m = Settings.minimumPowerOverKS;
        double[] kS = {m * (c * kSF + sn * kSS), m * (sn * kSF + c * kSS), m * Calibration.getDouble("pathmaker.kSTurn")};
        double energy = Settings.robotMassKg * (tw.vx * tw.vx + tw.vy * tw.vy + Math.pow(deg(tw.omega), 2) / 25);
        double scaling = Math.max(powerScaling, 0.05);

        double[] p = new double[3];
        boolean passed = false;
        for (int i = 0; i < 3; i++) {
            if (elapsedMs < delay[i]) { // axis not started yet: keep the previous power (rolling start)
                p[i] = last[i];
                continue;
            }
            boolean pass = s.passThrough && initialSign[i] != 0;
            double power;
            if (!pass) {
                if (Math.abs(e[i]) > reach[i]) {
                    power = Math.signum(e[i]);
                } else if (energy > Settings.energyThreshold) {
                    power = 0; // full braking
                } else {
                    power = e[i] / reach[i];
                    if (Settings.minimumPowerFix && Math.abs(e[i]) > zone[i]) {
                        power = Math.signum(e[i]) * Math.max(Math.abs(power), kS[i] / (i < 2 ? scaling : 1));
                    }
                }
            } else {
                if ((int) Math.signum(e[i]) != initialSign[i]) passed = true;
                power = Math.abs(e[i]) > reach[i] ? initialSign[i]
                        : initialSign[i] * Math.max(kS[i] / (i < 2 ? scaling : 1), Math.abs(e[i] / reach[i]));
            }
            double stepUp = Settings.powerStepUpPerSecond * dt;
            if (Math.abs(power) > Math.abs(last[i]) + stepUp) power = last[i] + Math.signum(power) * stepUp;
            p[i] = power;
            last[i] = power;
        }

        // share power by the sum of the axes (PathMaker 1's balancePower), then scale x and y
        double sum = Math.abs(p[0]) + Math.abs(p[1]) + Math.abs(p[2]);
        if (sum > 1) for (int i = 0; i < 3; i++) p[i] /= sum;
        p[0] *= powerScaling;
        p[1] *= powerScaling;

        boolean inZone = Math.abs(e[0]) < zone[0] && Math.abs(e[1]) < zone[1] && Math.abs(e[2]) < zone[2];
        boolean slow = Math.hypot(tw.vx, tw.vy) < Settings.minVelocityXY_inPerSec
                && Math.abs(deg(tw.omega)) < Settings.minVelocityTurn_degPerSec;
        atEnd = s.passThrough ? (passed || inZone) : inZone && (!Settings.speedCheckFix || slow);
        remaining = Math.hypot(e[0], e[1]);
        debug.put("err_x", e[0]);
        debug.put("err_y", e[1]);
        debug.put("err_h_deg", e[2]);
        debug.put("energy", energy);
        return toRobot(p[0], p[1], p[2], h);
    }

    /** PathMaker 1 has no position hold: it lets go at the end. */
    @Override
    protected DrivePowers hold(Pose target, MotionState state, double dt) {
        for (int i = 0; i < 3; i++) last[i] = 0;
        return DrivePowers.zero();
    }

    @Override
    public void reset() {
        super.reset();
    }

    private static double[] errors(StepSpec s, Pose pose) {
        return new double[]{s.x - pose.x(), s.y - pose.y(), Route.wrapDeg(s.h - deg(pose.heading()))};
    }

    private static double[] zones() {
        return new double[]{Settings.targetZoneXY_in, Settings.targetZoneXY_in, Settings.targetZoneTurn_deg};
    }
}
