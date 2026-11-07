package org.firstinspires.ftc.teamcode.calibration.steps;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.calibration.Calibration;
import org.firstinspires.ftc.teamcode.calibration.runner.Display;
import org.firstinspires.ftc.teamcode.calibration.runner.Inputs;
import org.firstinspires.ftc.teamcode.calibration.runner.Procedure;
import org.firstinspires.ftc.teamcode.calibration.runner.TuningOpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Measures what PathMaker needs and Pedro doesn't: the power that just starts the robot moving
 * (kS), how hard it stops with the motors braking, the top turning speed, and how much the
 * measured speed jitters while standing still. Uses the calibrated OctoQuad localizer.
 *
 * Every powered run stops at a distance limit and a time limit.
 */
public class DriveCharacterization extends Procedure {

    enum Axis { FORWARD, STRAFE, TURN }

    static final double RAMP_POWER_PER_S = 0.1, RAMP_MAX_POWER = 0.4, RAMP_MAX_S = 5;
    static final double BRAKE_MAX_S = 3;

    public DriveCharacterization() {
        super("PathMaker drive",
                "Start-up power, braking and turn speed for PathMaker. Needs calibrated odometry.");
    }

    @Override
    public void run() throws InterruptedException {
        if (!Calibration.allCalibrated("octoquad.xPodOffset", "octoquad.yPodOffset", "octoquad.headingScalar")) {
            abort("Calibrate the odometry first (OctoQuad step): this step measures with it.");
        }
        Inputs in = inputs("Test settings", "Each run stops at the distance limit. Make sure there is at least this "
                + "much room in the direction of each run, plus a tile around the robot.");
        Inputs.Field<Double> limit = in.d("Distance limit (in)").withDefault(48.0);
        Inputs.Field<Double> brakePower = in.d("Braking test power (0.3 to 1)").withDefault(0.7);
        Inputs.Field<Double> turnPower = in.d("Turning test power (0.3 to 1)").withDefault(0.6);
        awaitInputs(in);
        double lim = clamp(limit.get(), 18, 96);
        double bp = clamp(brakePower.get(), 0.3, 1), tp = clamp(turnPower.get(), 0.3, 1);

        double[] noise = runOpMode(new StandStill());
        double noiseXY = Math.max(3 * noise[0], 0.2), noiseTurn = Math.max(3 * noise[1], 0.5);
        result("pathmaker.stillNoiseXY", noiseXY);
        result("pathmaker.stillNoiseTurn", noiseTurn);

        double ksF = arrow("forward", new StartUp(Axis.FORWARD, lim, noiseXY));
        double ksS = arrow("left", new StartUp(Axis.STRAFE, lim, noiseXY));
        double ksT = arrow("turn", new StartUp(Axis.TURN, lim, noiseTurn));
        result("pathmaker.kSForward", ksF);
        result("pathmaker.kSStrafe", ksS);
        result("pathmaker.kSTurn", ksT);

        double[] bf = arrow("forward", new Braking(Axis.FORWARD, bp, lim, noiseXY));
        double[] bs = arrow("left", new Braking(Axis.STRAFE, bp, lim, noiseXY));
        double[] bt = arrow("turn", new Braking(Axis.TURN, tp, lim, noiseTurn));
        result("pathmaker.brakeDecelForward", bf[0]);
        result("pathmaker.brakeDecelStrafe", bs[0]);
        result("pathmaker.brakeDecelTurn", bt[0]);
        // Turn speed is measured at the test power; scale to full power above the start-up power.
        result("pathmaker.maxTurnSpeed", bt[1] * (1 - ksT) / Math.max(tp - ksT, 0.05));
        result("Braking from", String.format("%.0f in/s forward, %.0f in/s strafing, %.0f deg/s turning", bf[1], bs[1], bt[1]));
    }

    private <R> R arrow(String direction, TuningOpMode<R> op) throws InterruptedException {
        final Object[] out = new Object[1];
        withDisplay(Display.arrow(direction), () -> out[0] = runOpMode(op));
        @SuppressWarnings("unchecked") R r = (R) out[0];
        return r;
    }

    static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }

    /** Deceleration from a speed and the distance it took to stop: v² / (2 d). */
    static double deceleration(double speed, double distance) {
        return distance <= 1e-6 ? Double.NaN : speed * speed / (2 * distance);
    }

    // ------------------------------------------------------------------ runs

    /** Base: the calibrated localizer and drivetrain, readings along one axis, and safety limits. */
    abstract static class AxisRun<R> extends TuningOpMode<R> {
        final Axis axis;
        final double limit;
        Localizer localizer;
        Mecanum drive;
        private double lastHeading, turnedDeg;

        AxisRun(String title, String description, Axis axis, double limit) {
            super(title, description, false);
            this.axis = axis;
            this.limit = limit;
        }

        void setUp() {
            localizer = new OctoQuadLocalizer(hardwareMap, Constants.localizerConfig());
            drive = new Mecanum(hardwareMap, Constants.drivetrainConfig());
            localizer.setPose(Pose.zero());
            localizer.update();
            lastHeading = localizer.pose().heading();
            turnedDeg = 0;
        }

        /** Reads the localizer and adds up the turn, so turns past 180 degrees count correctly. */
        void update() {
            localizer.update();
            double h = localizer.pose().heading();
            turnedDeg += Math.toDegrees(Math.atan2(Math.sin(h - lastHeading), Math.cos(h - lastHeading)));
            lastHeading = h;
        }

        void power(double p) {
            switch (axis) {
                case FORWARD: drive.drive(new DrivePowers(p, 0, 0), false); break;
                case STRAFE: drive.drive(new DrivePowers(0, p, 0), false); break;
                default: drive.drive(new DrivePowers(0, 0, p), false);
            }
        }

        /** Speed along the axis: in/s, or deg/s for turning. */
        double speed() {
            Twist t = localizer.twist();
            switch (axis) {
                case FORWARD: return t.vx;
                case STRAFE: return t.vy;
                default: return Math.toDegrees(t.omega);
            }
        }

        /** Distance moved along the axis: in, or deg for turning. */
        double travelled() {
            Pose p = localizer.pose();
            return axis == Axis.TURN ? turnedDeg : Math.hypot(p.x(), p.y());
        }

        /** True if a run has gone too far. Turning is limited to two turns. */
        boolean tooFar() {
            return axis == Axis.TURN ? Math.abs(travelled()) > 720 : travelled() > limit;
        }

        void brake() {
            for (String m : new String[]{"mecanum.frontLeftName", "mecanum.frontRightName",
                    "mecanum.backLeftName", "mecanum.backRightName"}) {
                DcMotor motor = hardwareMap.get(DcMotor.class, Calibration.getString(m));
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor.setPower(0);
            }
        }

        void show(double power) {
            telemetry.addData("speed", speed());
            telemetry.addData("travelled", travelled());
            telemetry.addData("power", power);
            telemetry.update();
        }

        static String unit(Axis a) { return a == Axis.TURN ? "deg/s" : "in/s"; }

        static String room(Axis a) {
            switch (a) {
                case FORWARD: return "Make sure the space ahead of the robot is clear.";
                case STRAFE: return "Make sure the space to the robot's left is clear.";
                default: return "Make sure there is room for the robot to spin in place.";
            }
        }
    }

    /** Standing still for 2 s: RMS of the measured speeds, in/s and deg/s. */
    static final class StandStill extends AxisRun<double[]> {
        StandStill() {
            super("Standing still", "Leave the robot alone for 2 seconds. Nothing moves.", Axis.FORWARD, 1);
        }

        @Override
        protected double[] runTuningOpMode() throws InterruptedException {
            setUp();
            waitForStart();
            double sumXY = 0, sumTurn = 0;
            int n = 0;
            ElapsedTime t = new ElapsedTime();
            while (!isStopRequested() && t.seconds() < 2) {
                localizer.update();
                Twist tw = localizer.twist();
                sumXY += tw.vx * tw.vx + tw.vy * tw.vy;
                double w = Math.toDegrees(tw.omega);
                sumTurn += w * w;
                n++;
                telemetry.addData("speed in/s", Math.hypot(tw.vx, tw.vy));
                telemetry.addData("turn deg/s", w);
                telemetry.update();
            }
            return new double[]{Math.sqrt(sumXY / Math.max(n, 1)), Math.sqrt(sumTurn / Math.max(n, 1))};
        }
    }

    /** Raises power slowly until the robot moves. Returns that power (kS). */
    static final class StartUp extends AxisRun<Double> {
        final double threshold;

        StartUp(Axis axis, double limit, double noise) {
            super("Start-up power: " + axis.name().toLowerCase(),
                    "Power rises slowly from zero until the robot starts moving, then stops. " + room(axis),
                    axis, limit);
            // "Moving" means clearly above the standing-still noise.
            this.threshold = Math.max(3 * noise, axis == Axis.TURN ? 8 : 1.5);
        }

        @Override
        protected Double runTuningOpMode() throws InterruptedException {
            setUp();
            waitForStart();
            ElapsedTime t = new ElapsedTime();
            int above = 0;
            double p = 0;
            while (!isStopRequested()) {
                update();
                p = Math.min(RAMP_POWER_PER_S * t.seconds(), RAMP_MAX_POWER);
                power(p);
                show(p);
                above = Math.abs(speed()) > threshold ? above + 1 : 0;
                if (above >= 3) break; // moving for three readings in a row
                if (tooFar() || t.seconds() > RAMP_MAX_S) {
                    brake();
                    throw new org.firstinspires.ftc.teamcode.calibration.runner.CalibrationAbort(String.format(
                            "The robot didn't start moving below %.2f power. Check the drive motors.", p), false);
                }
            }
            brake();
            return p;
        }
    }

    /** Drives at a set power, then brakes. Returns {deceleration, speed when braking started}. */
    static final class Braking extends AxisRun<double[]> {
        final double testPower, noise;

        Braking(Axis axis, double testPower, double limit, double noise) {
            super("Braking: " + axis.name().toLowerCase(),
                    String.format("Drives at %.0f%% power, then brakes hard. ", testPower * 100) + room(axis),
                    axis, limit);
            this.testPower = testPower;
            this.noise = noise;
        }

        @Override
        protected double[] runTuningOpMode() throws InterruptedException {
            setUp();
            waitForStart();
            ElapsedTime t = new ElapsedTime();
            // Speed up for 60% of the limit (turning: up to 1.5 s).
            while (!isStopRequested()) {
                update();
                power(testPower);
                show(testPower);
                boolean far = axis == Axis.TURN ? t.seconds() > 1.5 : travelled() > 0.6 * limit;
                if (far || tooFar() || t.seconds() > BRAKE_MAX_S) break;
            }
            if (isStopRequested()) {
                brake();
                return new double[]{Double.NaN, Double.NaN};
            }
            double v0 = Math.abs(speed());
            double start = travelled();
            brake();
            t.reset();
            double lastStill = -1;
            while (!isStopRequested() && t.seconds() < BRAKE_MAX_S) {
                update();
                show(0);
                if (Math.abs(speed()) < noise) {
                    if (lastStill < 0) lastStill = t.seconds();
                    if (t.seconds() - lastStill > 0.15) break; // stopped
                } else {
                    lastStill = -1;
                }
            }
            double distance = Math.abs(travelled() - start);
            return new double[]{deceleration(v0, distance), v0};
        }
    }
}
