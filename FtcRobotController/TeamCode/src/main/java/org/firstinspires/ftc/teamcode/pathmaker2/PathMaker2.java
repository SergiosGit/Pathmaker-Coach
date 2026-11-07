package org.firstinspires.ftc.teamcode.pathmaker2;

/**
 * PathMaker 2's control law: drive straight at the target, as fast as the robot can still stop.
 *
 * Every number it needs is measured by the calibration page (calibration/), so there is nothing
 * to tune by hand for a first run:
 *  - top speed forward and sideways (Pedro Foresight step) and turning (PathMaker drive step);
 *  - braking deceleration with the motors braking (PathMaker drive step);
 *  - start-up power kS (PathMaker drive step).
 *
 * Each loop it picks a speed: the lowest of the top speed, the speed from which the robot can
 * still brake to a stop at the target (v = √(2·a·d)), and a gentle final approach (v = kP·d).
 * Power = kS + kV·speed + a little feedback on the measured speed. When the robot is faster than
 * that speed, the power along the path is cut to zero so the motors brake: the braking rate was
 * measured that way (with positive power the robot only coasts down, about 5 times slower).
 * The same for heading.
 * Translation is one 2D vector, so the robot drives a straight line (PathMaker 1 controls x and y
 * separately). Wheel powers are scaled by the largest wheel, and battery voltage is compensated.
 *
 * Plain Java with no hardware or Pedro classes: the bench adapter and the laptop simulator use it.
 * Units: inches, seconds, radians inside; degrees only in the parameters that say so.
 */
public final class PathMaker2 {

    /** Robot constants (from the calibration) and control settings. */
    public static final class Params {
        // measured
        public double kSForward = 0.10, kSStrafe = 0.14, kSTurn = 0.10;
        public double maxSpeedForward = 60, maxSpeedStrafe = 48;  // in/s
        public double maxTurnSpeedDeg = 300;                       // deg/s
        public double brakeForward = 150, brakeStrafe = 160;       // in/s²
        public double brakeTurnDeg = 1700;                         // deg/s²
        // settings
        /** Use this fraction of the measured braking, so the robot stops in time with margin. */
        public double brakeMargin = 0.6;
        /** Brake (power off) when this much faster than the planned speed, in/s. */
        public double brakeSlack = 2;
        /** Speed-up limit, in/s² and deg/s², so the wheels don't slip at the start. */
        public double accel = 150, accelTurnDeg = 1200;
        /** Final approach: speed per inch (per degree) of remaining error, 1/s. */
        public double approachGain = 4, approachGainTurn = 4;
        /** Feedback on speed: power per in/s (per deg/s) of speed error. */
        public double speedGain = 0.01, speedGainTurnDeg = 0.001;
        /** Done when within these and slower than these. */
        public double positionTolerance = 0.5, headingToleranceDeg = 1.0;
        public double stoppedSpeed = 1.0, stoppedTurnDeg = 3.0;
        /** A pass-through point counts as reached within this distance. */
        public double passRadius = 3;
        /** 1 = full speed; the benchmark's "matched" setting uses 0.6. */
        public double speedScale = 1;
        /** Battery compensation: powers are scaled by nominal / measured voltage. 0 turns it off. */
        public double nominalVoltage = 12.5;
    }

    /** One target: where to go, which heading, and whether to stop there. */
    public static final class Target {
        public final double x, y, heading; // heading in radians
        public final double startX, startY, startHeading;
        public final boolean linearHeading, tangentHeading, passThrough;

        public Target(double startX, double startY, double startHeading, double x, double y, double heading,
                      boolean linearHeading, boolean tangentHeading, boolean passThrough) {
            this.startX = startX;
            this.startY = startY;
            this.startHeading = startHeading;
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.linearHeading = linearHeading;
            this.tangentHeading = tangentHeading;
            this.passThrough = passThrough;
        }
    }

    /** Result of one loop. Powers are robot-relative: forward, left, counterclockwise. */
    public static final class Output {
        public double forward, strafe, turn;
        public boolean done;
        public double distance, headingErrorDeg, commandedSpeed;
    }

    private final Params p;
    private double cmdSpeed = 0, cmdTurn = 0; // ramped speed commands, carried across targets

    public PathMaker2(Params params) {
        this.p = params;
    }

    public Params params() { return p; }

    /** Forget the ramped speed commands, e.g. when a new route starts from standstill. */
    public void reset() {
        cmdSpeed = 0;
        cmdTurn = 0;
    }

    /**
     * One control step.
     * @param x,y,h  robot pose (in, in, rad)
     * @param vFwd,vLeft,omega  measured robot-relative velocity (in/s, in/s, rad/s)
     * @param dt     seconds since the last call
     * @param volts  battery voltage, or 0 if unknown
     * @param hold   true: keep the robot at the target forever (never "done")
     */
    public Output update(Target t, double x, double y, double h, double vFwd, double vLeft, double omega,
                         double dt, double volts, boolean hold) {
        Output o = new Output();
        dt = Math.max(0, Math.min(dt, 0.1));

        // ---- translation: one vector straight at the target
        double ex = t.x - x, ey = t.y - y;
        double d = Math.hypot(ex, ey);
        double ux = d > 1e-9 ? ex / d : 0, uy = d > 1e-9 ? ey / d : 0;
        // direction in robot frame (forward, left)
        double c = Math.cos(h), s = Math.sin(h);
        double uf = ux * c + uy * s, ul = -ux * s + uy * c;

        double vMaxDir = blend(uf, ul, p.maxSpeedForward, p.maxSpeedStrafe) * p.speedScale;
        double aDir = blend(uf, ul, p.brakeForward, p.brakeStrafe) * p.brakeMargin;
        double vWanted;
        if (t.passThrough && !hold) {
            vWanted = vMaxDir;
        } else {
            double stopDist = Math.max(d - 0.25 * p.positionTolerance, 0);
            vWanted = Math.min(vMaxDir, Math.min(Math.sqrt(2 * aDir * stopDist), p.approachGain * d));
        }
        // ramp up gently, slow down at once
        cmdSpeed = vWanted > cmdSpeed ? Math.min(vWanted, cmdSpeed + p.accel * dt) : vWanted;

        double cmdF = cmdSpeed * uf, cmdL = cmdSpeed * ul;
        boolean pushXY = d > p.positionTolerance;
        double kVF = (1 - p.kSForward) / p.maxSpeedForward, kVS = (1 - p.kSStrafe) / p.maxSpeedStrafe;
        double fwd, left;
        double vAlong = vFwd * uf + vLeft * ul;
        if (vAlong > cmdSpeed + p.brakeSlack) {
            // Too fast for the stopping distance: no power along the path, so the motors brake as
            // hard as the calibration measured (positive power only lets the robot coast down).
            // Only steer back onto the line sideways.
            double crossF = vFwd - vAlong * uf, crossL = vLeft - vAlong * ul;
            fwd = -p.speedGain * crossF;
            left = -p.speedGain * crossL;
        } else {
            fwd = (pushXY ? p.kSForward * uf : 0) + kVF * cmdF + p.speedGain * (cmdF - vFwd);
            left = (pushXY ? p.kSStrafe * ul : 0) + kVS * cmdL + p.speedGain * (cmdL - vLeft);
        }

        // ---- heading
        double hTarget = t.heading;
        if (t.tangentHeading && d > p.passRadius) {
            hTarget = Math.atan2(ey, ex);
        } else if (t.linearHeading) {
            double d0 = Math.hypot(t.x - t.startX, t.y - t.startY);
            double progress = d0 < 1e-6 ? 1 : clamp(1 - d / d0, 0, 1);
            hTarget = t.startHeading + wrap(t.heading - t.startHeading) * progress;
            if (d0 < 1e-6) hTarget = t.heading; // turn in place
        }
        double eh = wrap(hTarget - h);
        double ehDeg = Math.toDegrees(eh);
        double wMax = Math.toRadians(p.maxTurnSpeedDeg) * p.speedScale;
        double alpha = Math.toRadians(p.brakeTurnDeg) * p.brakeMargin;
        double wWanted = Math.signum(eh) * Math.min(wMax, Math.min(Math.sqrt(2 * alpha * Math.abs(eh)),
                p.approachGainTurn * Math.abs(eh)));
        double accelW = Math.toRadians(p.accelTurnDeg) * dt;
        // ramp up gently (from zero when the direction reverses), slow down at once
        if (Math.signum(wWanted) != Math.signum(cmdTurn)) cmdTurn = 0;
        cmdTurn = Math.abs(wWanted) > Math.abs(cmdTurn)
                ? cmdTurn + Math.signum(wWanted) * Math.min(accelW, Math.abs(wWanted) - Math.abs(cmdTurn))
                : wWanted;
        double kVT = (1 - p.kSTurn) / Math.toRadians(p.maxTurnSpeedDeg);
        boolean pushH = Math.abs(ehDeg) > p.headingToleranceDeg;
        double turn = (pushH ? p.kSTurn * Math.signum(eh) : 0) + kVT * cmdTurn
                + Math.toDegrees(cmdTurn - omega) * p.speedGainTurnDeg;

        // ---- battery, then scale by the largest wheel (mecanum wheel = forward ± strafe ± turn)
        double comp = p.nominalVoltage > 0 && volts > 6 ? clamp(p.nominalVoltage / volts, 0.8, 1.3) : 1;
        fwd *= comp;
        left *= comp;
        turn *= comp;
        double largest = Math.abs(fwd) + Math.abs(left) + Math.abs(turn);
        if (largest > 1) {
            fwd /= largest;
            left /= largest;
            turn /= largest;
        }

        o.forward = fwd;
        o.strafe = left;
        o.turn = turn;
        o.distance = d;
        o.headingErrorDeg = ehDeg;
        o.commandedSpeed = cmdSpeed;
        if (hold) {
            o.done = false;
        } else if (t.passThrough) {
            // reached, or already past the point along the line from the start
            double segX = t.x - t.startX, segY = t.y - t.startY;
            o.done = d < p.passRadius || (segX * ex + segY * ey) < 0;
        } else {
            double speed = Math.hypot(vFwd, vLeft);
            o.done = d < p.positionTolerance && Math.abs(ehDeg) < p.headingToleranceDeg
                    && speed < p.stoppedSpeed && Math.abs(Math.toDegrees(omega)) < p.stoppedTurnDeg;
        }
        return o;
    }

    /** A value that differs forward and sideways, in the direction (uf, ul): 1 / (|uf|/a + |ul|/b). */
    static double blend(double uf, double ul, double forward, double sideways) {
        double inv = Math.abs(uf) / forward + Math.abs(ul) / sideways;
        return inv < 1e-12 ? forward : 1 / inv;
    }

    static double wrap(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }

    static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
