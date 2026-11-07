package org.firstinspires.ftc.teamcode.bench;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.util.Locale;

/**
 * One CSV row per loop, to /sdcard/FIRST/bench/<test>_<engine>_<speed>_<run>.csv on the robot.
 * Columns as in docs/PATHING-BENCHMARK-PLAN.md section 4. Pedro coordinates for every engine.
 */
public class BenchLogger implements AutoCloseable {

    public static final String HEADER = "t_s,engine,test,run,step,target_x,target_y,target_h_deg,"
            + "pose_x,pose_y,pose_h_deg,vel_x,vel_y,vel_h_degs,power_fwd,power_str,power_turn,"
            + "battery_v,loop_ms,busy,at_parametric_end,shooter_target_rpm,shooter_rpm";

    private final BufferedWriter out;
    private final String engine, test;
    private final int run;
    public final File file;

    public BenchLogger(File dir, String test, String engine, String speed, int run) throws IOException {
        if (!dir.exists() && !dir.mkdirs()) throw new IOException("cannot create " + dir);
        this.file = new File(dir, String.format(Locale.US, "%s_%s_%s_%02d.csv", test, engine, speed, run));
        this.out = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(file), StandardCharsets.UTF_8));
        this.engine = engine;
        this.test = test;
        this.run = run;
        out.write(HEADER);
        out.newLine();
    }

    /** Field velocities in in/s and deg/s; powers robot-relative. NaN for unknown shooter values. */
    public void row(double t, int step, double[] target, double x, double y, double hDeg,
                    double vx, double vy, double vhDeg, double pf, double ps, double pt,
                    double volts, double loopMs, boolean busy, boolean atEnd, double shooterTarget, double shooterRpm)
            throws IOException {
        out.write(String.format(Locale.US,
                "%.4f,%s,%s,%d,%d,%.3f,%.3f,%.2f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f,%.3f,%.3f,%.3f,%.2f,%.2f,%d,%d,%s,%s",
                t, engine, test, run, step, target[0], target[1], target[2], x, y, hDeg, vx, vy, vhDeg,
                pf, ps, pt, volts, loopMs, busy ? 1 : 0, atEnd ? 1 : 0, num(shooterTarget), num(shooterRpm)));
        out.newLine();
    }

    private static String num(double v) { return Double.isNaN(v) ? "" : String.format(Locale.US, "%.0f", v); }

    @Override
    public void close() throws IOException { out.close(); }
}
