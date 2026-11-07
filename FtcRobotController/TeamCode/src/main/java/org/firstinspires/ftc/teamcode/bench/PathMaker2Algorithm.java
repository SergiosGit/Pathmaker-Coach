package org.firstinspires.ftc.teamcode.bench;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;

import org.firstinspires.ftc.teamcode.calibration.Calibration;
import org.firstinspires.ftc.teamcode.pathmaker2.PathMaker2;

import java.util.function.DoubleSupplier;

/** PathMaker 2 inside Pedro's Follower, with the robot constants from the calibration. */
public class PathMaker2Algorithm extends PathMakerAlgorithm {

    /** Settings that are choices rather than measurements. Live-editable in Panels. */
    @Configurable
    public static class Settings {
        public static double brakeMargin = 0.75, brakeSlack = 2, brakeMinSpeed = 20;
        public static double accel = 250, accelTurnDeg = 1200;
        public static double approachGain = 4, approachGainTurn = 4;
        public static double speedGain = 0.01, speedGainTurnDeg = 0.001;
        public static double positionTolerance = 0.5, headingToleranceDeg = 1;
        public static double passRadius = 3;
        public static double nominalVoltage = 12.5;
    }

    private final PathMaker2 controller;
    private final DoubleSupplier volts;
    private PathMaker2.Target target;

    public PathMaker2Algorithm(double speedScale, DoubleSupplier volts) {
        this.controller = new PathMaker2(params(speedScale));
        this.volts = volts;
    }

    /** The measured values from the calibration plus the settings above. */
    public static PathMaker2.Params params(double speedScale) {
        PathMaker2.Params p = new PathMaker2.Params();
        p.kSForward = Calibration.getDouble("pathmaker.kSForward");
        p.kSStrafe = Calibration.getDouble("pathmaker.kSStrafe");
        p.kSTurn = Calibration.getDouble("pathmaker.kSTurn");
        p.maxSpeedForward = Calibration.getDouble("foresight.maxAchievableForwardVelocity");
        p.maxSpeedStrafe = Calibration.getDouble("foresight.maxAchievableStrafeVelocity");
        p.maxTurnSpeedDeg = Calibration.getDouble("pathmaker.maxTurnSpeed");
        p.brakeForward = Calibration.getDouble("pathmaker.brakeDecelForward");
        p.brakeStrafe = Calibration.getDouble("pathmaker.brakeDecelStrafe");
        p.brakeTurnDeg = Calibration.getDouble("pathmaker.brakeDecelTurn");
        p.stoppedSpeed = Math.max(1.0, 3 * Calibration.getDouble("pathmaker.stillNoiseXY"));
        p.stoppedTurnDeg = Math.max(3.0, 3 * Calibration.getDouble("pathmaker.stillNoiseTurn"));
        p.brakeMargin = Settings.brakeMargin;
        p.brakeSlack = Settings.brakeSlack;
        p.brakeMinSpeed = Settings.brakeMinSpeed;
        p.accel = Settings.accel;
        p.accelTurnDeg = Settings.accelTurnDeg;
        p.approachGain = Settings.approachGain;
        p.approachGainTurn = Settings.approachGainTurn;
        p.speedGain = Settings.speedGain;
        p.speedGainTurnDeg = Settings.speedGainTurnDeg;
        p.positionTolerance = Settings.positionTolerance;
        p.headingToleranceDeg = Settings.headingToleranceDeg;
        p.passRadius = Settings.passRadius;
        p.nominalVoltage = Settings.nominalVoltage;
        p.speedScale = speedScale;
        return p;
    }

    @Override
    protected void startStep(StepSpec s, MotionState state) {
        target = new PathMaker2.Target(s.startX, s.startY, Math.toRadians(s.startH), s.x, s.y, Math.toRadians(s.h),
                s.heading == Route.Heading.LINEAR, s.heading == Route.Heading.TANGENT, s.passThrough);
    }

    @Override
    protected DrivePowers step(StepSpec s, MotionState state, double elapsedMs, double dt) {
        PathMaker2.Output o = run(target, state, dt, false);
        atEnd = o.done;
        return new DrivePowers(o.forward, o.strafe, o.turn);
    }

    @Override
    protected DrivePowers hold(Pose pose, MotionState state, double dt) {
        PathMaker2.Target t = new PathMaker2.Target(pose.x(), pose.y(), pose.heading(), pose.x(), pose.y(),
                pose.heading(), false, false, false);
        PathMaker2.Output o = run(t, state, dt, true);
        return new DrivePowers(o.forward, o.strafe, o.turn);
    }

    private PathMaker2.Output run(PathMaker2.Target t, MotionState state, double dt, boolean hold) {
        Pose p = state.pose();
        Twist tw = state.twist();
        PathMaker2.Output o = controller.update(t, p.x(), p.y(), p.heading(), tw.vx, tw.vy, tw.omega, dt,
                volts.getAsDouble(), hold);
        remaining = o.distance;
        debug.put("err_xy", o.distance);
        debug.put("err_h_deg", o.headingErrorDeg);
        debug.put("cmd_speed", o.commandedSpeed);
        return o;
    }

    @Override
    public void reset() {
        super.reset();
    }
}
