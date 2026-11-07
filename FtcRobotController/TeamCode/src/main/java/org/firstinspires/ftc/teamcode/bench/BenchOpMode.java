package org.firstinspires.ftc.teamcode.bench;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.io.File;

/**
 * Runs one benchmark test (docs/PATHING-BENCHMARK-PLAN.md) and writes one CSV per run.
 * Choose engine, test, speed and run number during INIT with gamepad 1's dpad (shown on the
 * Driver Hub), or in Panels (Configurables → BenchOpMode.Settings); they are read when ▶ is
 * pressed. Place the robot at (72, 24) facing up the field.
 */
@Autonomous(name = "Bench", group = "bench")
public class BenchOpMode extends LinearOpMode {

    public enum Speed { MAX, MATCHED }

    @Configurable
    public static class Settings {
        public static BenchEngine.Kind engine = BenchEngine.Kind.PEDRO;
        public static BenchRoutes.Test test = BenchRoutes.Test.A1;
        public static Speed speed = Speed.MAX;
        public static int run = 1;
        /** Keep logging this long after the route ends (holding, except PathMaker 1). */
        public static double afterSeconds = 2;
        /** Safety: stop the motors if the route takes longer than this. */
        public static double timeoutSeconds = 20;
        public static double matchedSpeed = 0.6;
    }

    /** Dpad up/down chooses the line, left/right changes it. Returns the selected line. */
    private int chooseWithGamepad(int row) {
        if (gamepad1.dpadUpWasPressed()) row = (row + 3) % 4;
        if (gamepad1.dpadDownWasPressed()) row = (row + 1) % 4;
        int step = gamepad1.dpadRightWasPressed() ? 1 : gamepad1.dpadLeftWasPressed() ? -1 : 0;
        if (step != 0) {
            switch (row) {
                case 0: Settings.engine = cycle(BenchEngine.Kind.values(), Settings.engine, step); break;
                case 1: Settings.test = cycle(BenchRoutes.Test.values(), Settings.test, step); break;
                case 2: Settings.speed = cycle(Speed.values(), Settings.speed, step); break;
                default: Settings.run = Math.max(1, Settings.run + step);
            }
        }
        return row;
    }

    private static <E extends Enum<E>> E cycle(E[] all, E current, int step) {
        return all[(current.ordinal() + step + all.length) % all.length];
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry tele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        final double[] volts = {battery.getVoltage()};

        OctoQuadLocalizer localizer = new OctoQuadLocalizer(hardwareMap, Constants.localizerConfig());
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig());
        localizer.setPose(new Pose(BenchRoutes.START_X, BenchRoutes.START_Y, Math.toRadians(BenchRoutes.START_H)));

        int row = 0;
        while (!isStarted() && !isStopRequested()) {
            localizer.update();
            Pose p = localizer.pose();
            row = chooseWithGamepad(row);
            tele.addLine("Gamepad 1: dpad ▲▼ picks a line, ◀▶ changes it. Then press ▶.");
            String[] names = {"engine", "test", "speed", "run"};
            Object[] values = {Settings.engine, Settings.test, Settings.speed, Settings.run};
            for (int i = 0; i < names.length; i++) {
                tele.addData((i == row ? "▶ " : "   ") + names[i], values[i]);
            }
            tele.addData("pose (should be 72, 24, 90)", "%.1f, %.1f, %.1f", p.x(), p.y(), Math.toDegrees(p.heading()));
            tele.addData("battery", "%.2f V", battery.getVoltage());
            tele.update();
            sleep(50);
        }
        if (isStopRequested()) return;

        double scale = Settings.speed == Speed.MAX ? 1 : Settings.matchedSpeed;
        BenchEngine engine = new BenchEngine(Settings.engine, localizer, drivetrain, scale,
                Constants::foresightConfig, () -> volts[0]);
        Route route = BenchRoutes.get(Settings.test);
        engine.setPose(route.startX, route.startY, route.startHeadingDeg);

        try (BenchLogger log = new BenchLogger(new File("/sdcard/FIRST/bench"), Settings.test.name(),
                Settings.engine.name(), Settings.speed.name(), Settings.run)) {
            engine.start(route);
            long t0 = System.nanoTime(), last = t0;
            double doneAt = -1;
            int loops = 0;
            while (opModeIsActive()) {
                long now = System.nanoTime();
                double dt = (now - last) / 1e9, t = (now - t0) / 1e9;
                last = now;
                if (loops % 10 == 0) volts[0] = battery.getVoltage(); // not in the bulk read, so not every loop
                engine.update(dt);

                Pose p = engine.pose();
                int step = engine.routeStep();
                double[] target = step >= 0 && step < route.steps.size()
                        ? new double[]{route.steps.get(step).x, route.steps.get(step).y, route.steps.get(step).headingDeg}
                        : route.endPose();
                com.pedropathing.math.Velocity v = engine.follower().velocity();
                log.row(t, step, target, p.x(), p.y(), Math.toDegrees(p.heading()), v.vx, v.vy, Math.toDegrees(v.omega),
                        engine.lastPowers().forward(), engine.lastPowers().strafe(), engine.lastPowers().turn(),
                        volts[0], dt * 1000, !engine.done(), engine.atParametricEnd(), Double.NaN, Double.NaN);

                if (engine.done() && doneAt < 0) doneAt = t;
                if (doneAt >= 0 && t - doneAt > Settings.afterSeconds) break;
                if (t > Settings.timeoutSeconds) {
                    tele.addLine("Timed out: stopping");
                    break;
                }
                if (loops++ % 4 == 0) {
                    tele.addData("engine / test", "%s / %s", Settings.engine, Settings.test);
                    tele.addData("step", step);
                    tele.addData("pose", "%.1f, %.1f, %.1f", p.x(), p.y(), Math.toDegrees(p.heading()));
                    tele.addData("loop_ms", dt * 1000);
                    tele.addData("done at", doneAt < 0 ? "-" : String.format("%.2f s", doneAt));
                    tele.update();
                }
            }
            drivetrain.stop();
            tele.addData("log", log.file.getPath());
            tele.addData("time to done", doneAt < 0 ? "not done" : String.format("%.2f s", doneAt));
            tele.update();
        } catch (java.io.IOException e) {
            drivetrain.stop();
            tele.addData("could not write the log", e.getMessage());
            tele.update();
        }
        while (opModeIsActive()) sleep(50);
    }
}
