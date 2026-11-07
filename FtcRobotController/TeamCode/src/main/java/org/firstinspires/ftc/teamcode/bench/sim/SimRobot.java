package org.firstinspires.ftc.teamcode.bench.sim;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;

import java.util.HashMap;
import java.util.Map;

/**
 * A simple simulated mecanum robot for laptop tests of the engines: it is both Pedro's Localizer
 * and Drivetrain. Each axis (forward, sideways, turning) follows the power with a lag; below the
 * start-up power kS it brakes with the measured braking deceleration.
 */
public class SimRobot implements Localizer, Drivetrain {

    public double kSF = 0.10, kSS = 0.14, kST = 0.10;
    public double vMaxF = 62, vMaxS = 48, wMaxDeg = 330;   // in/s, deg/s
    public double brakeF = 149, brakeS = 159, brakeTDeg = 1730;
    /** Slowing down while power still drives the wheels forward: about the coasting rate. */
    public double coastF = 31, coastS = 57, coastTDeg = 400;
    public double lag = 0.12;                               // s
    public double volts = 12.5, nominalVolts = 12.5;

    private double x, y, h, vf, vs, w;  // field pose; robot-relative velocity (in/s, rad/s)
    private DrivePowers powers = DrivePowers.zero();

    /** Advances the simulation by dt seconds with the last powers. */
    public void step(double dt) {
        double f = powers.forward(), s = powers.strafe(), t = powers.turn();
        double sum = Math.abs(f) + Math.abs(s) + Math.abs(t);
        if (sum > 1) { f /= sum; s /= sum; t /= sum; }
        double v = volts / nominalVolts;
        vf = axis(vf, f * v, kSF, vMaxF, brakeF, coastF, dt);
        vs = axis(vs, s * v, kSS, vMaxS, brakeS, coastS, dt);
        w = Math.toRadians(axis(Math.toDegrees(w), t * v, kST, wMaxDeg, brakeTDeg, coastTDeg, dt));
        double c = Math.cos(h), sn = Math.sin(h);
        x += (vf * c - vs * sn) * dt;
        y += (vf * sn + vs * c) * dt;
        h = Math.atan2(Math.sin(h + w * dt), Math.cos(h + w * dt));
    }

    private double axis(double vel, double p, double kS, double vMax, double brake, double coast, double dt) {
        if (Math.abs(p) < kS) { // not enough power to drive: brake toward zero
            double dv = brake * dt;
            return Math.abs(vel) <= dv ? 0 : vel - Math.signum(vel) * dv;
        }
        double target = Math.signum(p) * (Math.abs(p) - kS) / (1 - kS) * vMax;
        double next = vel + (target - vel) * Math.min(dt / lag, 1);
        // Slowing down: with power still pushing the same way the wheels turn, the robot only
        // coasts down (seen on the bay-b bot); reverse power brakes at most as hard as braking.
        if (Math.abs(next) < Math.abs(vel)) {
            double limit = (Math.signum(p) == Math.signum(vel) ? coast : brake) * dt;
            next = vel + Math.max(-limit, Math.min(limit, next - vel));
        }
        return next;
    }

    // ---- Localizer
    @Override public void setPose(Pose p) { x = p.x(); y = p.y(); h = p.heading(); }
    @Override public MotionState state() { return MotionState.ofTwist(new Pose(x, y, h), new Twist(vf, vs, w)); }
    @Override public void update() { }
    @Override public void reset() { x = y = h = vf = vs = w = 0; }

    // ---- Drivetrain
    @Override public void drive(DrivePowers p, boolean manual) { powers = p; }
    @Override public void stop() { powers = DrivePowers.zero(); }
    @Override public void stop(boolean brake) { powers = DrivePowers.zero(); }
    @Override public Map<String, Object> debug() { return new HashMap<>(); }

    /** As Pedro's Mecanum: the largest s in [0, 1] keeping every wheel of a + s·b within ±1. */
    @Override
    public double maxScaling(DrivePowers a, DrivePowers b) {
        double[] wa = wheels(a), wb = wheels(b);
        double s = 1;
        for (int i = 0; i < 4; i++) {
            if (Math.abs(wb[i]) < 1e-9) continue;
            double hi = Math.max((1 - wa[i]) / wb[i], (-1 - wa[i]) / wb[i]);
            s = Math.min(s, hi);
        }
        return Math.max(0, Math.min(1, s));
    }

    /** As Pedro's Mecanum: speed in direction θ blended from forward and sideways speeds. */
    @Override
    public double interpolateVelocity(double forward, double sideways, double theta) {
        return 1 / (Math.abs(Math.cos(theta)) / forward + Math.abs(Math.sin(theta)) / sideways);
    }

    private static double[] wheels(DrivePowers p) {
        double f = p.forward(), s = p.strafe(), t = p.turn();
        return new double[]{f - s - t, f + s - t, f + s + t, f - s + t};
    }
}
