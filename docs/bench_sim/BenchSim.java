import org.firstinspires.ftc.teamcode.bench.BenchEngine;
import org.firstinspires.ftc.teamcode.bench.BenchLogger;
import org.firstinspires.ftc.teamcode.bench.BenchRoutes;
import org.firstinspires.ftc.teamcode.bench.Route;
import org.firstinspires.ftc.teamcode.bench.sim.SimRobot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.math.Pose;

import java.util.Locale;

/**
 * Laptop check of the three engines on a simulated bay-b bot (constants from the calibration).
 * Runs every benchmark route at both speeds and prints time to done, end error, overshoot and
 * how far the robot strays from the straight line. A simulation can only catch gross mistakes
 * (wrong signs, routes that never finish, unstable control); the robot benchmark decides.
 *
 * From this folder, with the TeamCode classes and the FTC + Pedro jars on the classpath:
 *   javac -cp <classpath> BenchSim.java && java -cp .:<classpath> BenchSim [csv-folder]
 * With a folder, every run is also logged as CSV exactly like the robot does, for analyze.py.
 */
public class BenchSim {
    static final double DT = 0.012, MAX_S = 15;

    static java.io.File csvDir;

    public static void main(String[] args) throws Exception {
        if (args.length > 0) csvDir = new java.io.File(args[0]);
        System.out.println("test engine     speed    done_s  end_err_in end_err_deg overshoot_in offline_in");
        for (BenchRoutes.Test t : BenchRoutes.Test.values()) {
            if (t == BenchRoutes.Test.HOLD) continue;
            for (BenchEngine.Kind k : BenchEngine.Kind.values()) {
                for (double scale : new double[]{1, 0.6}) {
                    System.out.println(run(t, k, scale));
                }
            }
        }
    }

    static String run(BenchRoutes.Test test, BenchEngine.Kind kind, double scale) throws Exception {
        Route r = BenchRoutes.get(test);
        SimRobot sim = new SimRobot();
        BenchEngine e = new BenchEngine(kind, sim, sim, scale, Constants::foresightConfig, () -> 12.5);
        e.setPose(r.startX, r.startY, r.startHeadingDeg);
        e.start(r);
        double[] end = r.endPose();
        double doneAt = -1, overshoot = 0, offline = 0;
        // direction of the last step, for overshoot
        double[] a = r.startOf(r.steps.size() - 1);
        double dx = end[0] - a[0], dy = end[1] - a[1], len = Math.hypot(dx, dy);
        BenchLogger log = csvDir == null ? null
                : new BenchLogger(csvDir, test.name(), kind.name(), scale == 1 ? "MAX" : "MATCHED", 1);
        for (double t = 0; t < MAX_S; t += DT) {
            e.update(DT);
            sim.step(DT);
            Pose p = e.pose();
            if (log != null) {
                int step = e.routeStep();
                double[] tg = step >= 0 && step < r.steps.size()
                        ? new double[]{r.steps.get(step).x, r.steps.get(step).y, r.steps.get(step).headingDeg} : end;
                com.pedropathing.math.Velocity v = e.follower().velocity();
                log.row(t, step, tg, p.x(), p.y(), Math.toDegrees(p.heading()), v.vx, v.vy, Math.toDegrees(v.omega),
                        e.lastPowers().forward(), e.lastPowers().strafe(), e.lastPowers().turn(), 12.5, DT * 1000,
                        !e.done(), e.atParametricEnd(), Double.NaN, Double.NaN);
            }
            if (len > 1e-6) overshoot = Math.max(overshoot, ((p.x() - end[0]) * dx + (p.y() - end[1]) * dy) / len);
            if (!r.steps.get(0).isCurve() && r.steps.size() == 1 && len > 1e-6) {
                offline = Math.max(offline, Math.abs(((p.x() - a[0]) * dy - (p.y() - a[1]) * dx) / len));
            }
            if (e.done() && doneAt < 0) doneAt = t;
            if (doneAt >= 0 && t > doneAt + 1) break;
        }
        if (log != null) log.close();
        Pose p = e.pose();
        double err = Math.hypot(p.x() - end[0], p.y() - end[1]);
        double errH = Route.wrapDeg(Math.toDegrees(p.heading()) - end[2]);
        return String.format(Locale.US, "%-4s %-10s %-7s %7s %10.2f %11.2f %12.2f %10s",
                test, kind, scale == 1 ? "MAX" : "MATCHED", doneAt < 0 ? "never" : String.format(Locale.US, "%.2f", doneAt),
                err, errH, overshoot, r.steps.size() == 1 && !r.steps.get(0).isCurve() && len > 1e-6
                        ? String.format(Locale.US, "%.2f", offline) : "-");
    }
}
