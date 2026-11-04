import java.io.File;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

/**
 * Laptop study of the ball's flight from the shooter to the goal window. It answers:
 *   - What launch speed and angle make the ball enter the window straight on (normal to it)?
 *   - How much speed error and angle error can a shot have and still score?
 *   - With realistic random errors, which speed and angle score most often, at each distance?
 *   - If the hood angle is fixed, which single angle works best over a range of distances?
 *
 * The model is a sphere flying through air, with gravity, air drag and (optionally) lift from
 * backspin. The ball scores when it crosses the window plane on the way down and, seen along its
 * direction of travel, the whole ball fits inside the triangle (it does not touch the frame).
 * Read docs/SHOOTER-TRAJECTORY.md for the explanation and results.
 *
 * Estimates: ball mass, drag coefficient and lift coefficient. Weigh a ball and change Ball.kg.
 * The run takes a few minutes, because it flies a few million simulated balls.
 *
 * Run it from the docs/shooter_sim folder (needs only a JDK):
 *   javac -d out TrajectorySim.java
 *   java -cp out TrajectorySim            prints the tables used in the document
 *   java -cp out TrajectorySim out/traj   also writes CSV files for the plots
 */
public class TrajectorySim {

    // ---- Geometry, measured from the point where the ball leaves the shooter ----
    static final double WINDOW_HEIGHT_M = 1.7;    // window centre is this far above the shooter exit
    static final double WINDOW_LEAN_DEG = 30;     // window plane leans back this far from vertical
    static final double WINDOW_WIDTH_M = 0.53;    // triangle base
    static final double WINDOW_SLANT_M = 0.30;    // triangle height, measured along the leaning plane

    // ---- Ball: 2.8 inch whiffle ball ----
    static final double BALL_RADIUS_M = 2.8 * 0.0254 / 2;
    static final double AREA_M2 = Math.PI * BALL_RADIUS_M * BALL_RADIUS_M;
    static final double AIR_KG_M3 = 1.2;
    static final double G = 9.81;

    // ---- Shooter: ball centre leaves at (efficiency x half the wheel surface speed) ----
    static final double WHEEL_RADIUS_M = 0.036;   // 72 mm Rhino wheel

    static class Ball {
        double kg = 0.035;      // same estimate as ShooterSim. Weigh a real ball!
        double cd = 0.5;        // drag coefficient: the holes make a whiffle ball draggier than a smooth ball
        double cl = 0.0;        // lift coefficient from backspin. 0 = ignore spin
        boolean apexUp = true;  // true: point of the triangle at the top, base at the bottom
        String name;

        Ball(String name) { this.name = name; }
        Ball copy(String n) { Ball b = new Ball(n); b.kg = kg; b.cd = cd; b.cl = cl; b.apexUp = apexUp; return b; }
    }

    /** What happened to one shot. */
    static class Shot {
        boolean crossed;        // reached the window plane from the front, going down
        boolean reached;        // reached the window plane from the front, going up or down
        double s;               // where it crossed, along the plane from the window centre (m, + = up the slant)
        double entryDeg;        // angle below horizontal at the crossing
        double offNormalDeg;    // angle between the ball's path and the window's normal (0 = straight on)
        double marginM;         // clearance between ball and nearest window edge (negative = hits the frame)
        boolean scores() { return crossed && marginM >= 0; }
    }

    // =====================================================================================
    //  Flight
    // =====================================================================================

    /**
     * Fly one ball and see what it does at the window.
     * @param distanceM horizontal distance from the shooter exit to the window centre
     * @param lateralM  sideways miss at the window (for aiming tolerance), 0 = centred
     */
    static Shot fly(Ball b, double distanceM, double speed, double launchDeg, double lateralM, List<double[]> path) {
        double lean = Math.toRadians(WINDOW_LEAN_DEG);
        // Unit vector pointing into the goal, perpendicular to the window plane.
        double nx = Math.cos(lean), nz = -Math.sin(lean);
        // Unit vector up along the window plane (it leans away from the shooter).
        double ux = Math.sin(lean), uz = Math.cos(lean);

        double kDrag = 0.5 * AIR_KG_M3 * b.cd * AREA_M2 / b.kg;
        double kLift = 0.5 * AIR_KG_M3 * b.cl * AREA_M2 / b.kg;

        double x = 0, z = 0;
        double vx = speed * Math.cos(Math.toRadians(launchDeg));
        double vz = speed * Math.sin(Math.toRadians(launchDeg));
        double dt = 0.001;
        double side = nx * (x - distanceM) + nz * (z - WINDOW_HEIGHT_M); // < 0: in front of the window plane
        boolean wasInFront = side < 0;

        Shot shot = new Shot();
        for (double t = 0; t < 3.0; t += dt) {
            if (path != null) path.add(new double[]{x, z});
            // Midpoint (RK2) step: accurate enough at 1 ms and easy to read.
            double[] a1 = accel(vx, vz, kDrag, kLift);
            double mx = x + vx * dt / 2, mz = z + vz * dt / 2;
            double mvx = vx + a1[0] * dt / 2, mvz = vz + a1[1] * dt / 2;
            double[] a2 = accel(mvx, mvz, kDrag, kLift);
            double nxp = x + mvx * dt, nzp = z + mvz * dt;
            double nvx = vx + a2[0] * dt, nvz = vz + a2[1] * dt;

            double newSide = nx * (nxp - distanceM) + nz * (nzp - WINDOW_HEIGHT_M);
            if (wasInFront && newSide >= 0) {
                // Crossed the plane from the front. Interpolate to the crossing point.
                double f = -side / (newSide - side);
                double cx = x + f * (nxp - x), cz = z + f * (nzp - z);
                double cvx = vx + f * (nvx - vx), cvz = vz + f * (nvz - vz);
                shot.reached = true;
                shot.s = ux * (cx - distanceM) + uz * (cz - WINDOW_HEIGHT_M);
                if (cvz < 0) { // must be on the way down
                    shot.crossed = true;
                    shot.entryDeg = Math.toDegrees(Math.atan2(-cvz, cvx));
                    double cos = (cvx * nx + cvz * nz) / Math.hypot(cvx, cvz);
                    shot.offNormalDeg = Math.toDegrees(Math.acos(Math.min(1, cos)));
                    shot.marginM = margin(b, shot.s, lateralM, cos);
                    if (path != null) path.add(new double[]{cx, cz});
                } else {
                    shot.marginM = -1;
                }
                return shot;
            }
            if (newSide < 0) wasInFront = true;
            side = newSide;
            x = nxp; z = nzp; vx = nvx; vz = nvz;
            if (z < -0.5) break; // fell to the floor
        }
        shot.marginM = -1;
        return shot;
    }

    /** Gravity, drag (against the motion) and backspin lift (at right angles to the motion, upward). */
    static double[] accel(double vx, double vz, double kDrag, double kLift) {
        double v = Math.hypot(vx, vz);
        return new double[]{
                -kDrag * v * vx - kLift * v * vz,
                -G - kDrag * v * vz + kLift * v * vx};
    }

    /**
     * Clearance between the ball and the window frame. Looking along the ball's path, the window
     * looks shorter by cos(offNormal) in the slant direction (like a door seen from the side), while
     * the ball still looks like a full circle. The ball fits if its centre is at least one radius
     * inside every edge of that shortened triangle.
     */
    static double margin(Ball b, double s, double lateral, double cosOffNormal) {
        // Triangle in window coordinates (lateral, slant), with the centroid at (0, 0).
        double base = b.apexUp ? -WINDOW_SLANT_M / 3 : WINDOW_SLANT_M / 3;
        double apex = b.apexUp ? 2 * WINDOW_SLANT_M / 3 : -2 * WINDOW_SLANT_M / 3;
        double[][] v = {
                {-WINDOW_WIDTH_M / 2, base * cosOffNormal},
                {WINDOW_WIDTH_M / 2, base * cosOffNormal},
                {0, apex * cosOffNormal}};
        double px = lateral, py = s * cosOffNormal;
        double inside = Double.MAX_VALUE;
        for (int i = 0; i < 3; i++) {
            double[] p = v[i], q = v[(i + 1) % 3], r = v[(i + 2) % 3];
            double ex = q[0] - p[0], ey = q[1] - p[1], len = Math.hypot(ex, ey);
            double d = (ex * (py - p[1]) - ey * (px - p[0])) / len;
            double dr = (ex * (r[1] - p[1]) - ey * (r[0] - p[0])) / len; // sign of the inside
            inside = Math.min(inside, Math.signum(dr) * d);
        }
        return inside - BALL_RADIUS_M;
    }

    // =====================================================================================
    //  Searches built on fly()
    // =====================================================================================

    /** The launch that enters exactly straight on, through the window centre. Returns {speed, angle}. */
    static double[] normalEntry(Ball b, double d) {
        // Start from the no-air answer, then correct for drag with a few Newton steps.
        double tan = Math.tan(Math.toRadians(WINDOW_LEAN_DEG));
        double t = Math.sqrt(2 * (WINDOW_HEIGHT_M + d * tan) / G);
        double vx = d / t, vz = G * t - d * tan / t;
        double speed = Math.hypot(vx, vz), angle = Math.toDegrees(Math.atan2(vz, vx));
        for (int it = 0; it < 30; it++) {
            double[] r = residual(b, d, speed, angle);
            double h = 1e-4;
            double[] rs = residual(b, d, speed + h, angle), ra = residual(b, d, speed, angle + h);
            double j11 = (rs[0] - r[0]) / h, j12 = (ra[0] - r[0]) / h;
            double j21 = (rs[1] - r[1]) / h, j22 = (ra[1] - r[1]) / h;
            double det = j11 * j22 - j12 * j21;
            if (Math.abs(det) < 1e-12) break;
            speed -= (j22 * r[0] - j12 * r[1]) / det;
            angle -= (-j21 * r[0] + j11 * r[1]) / det;
            if (Math.abs(r[0]) < 1e-5 && Math.abs(r[1]) < 1e-4) break;
        }
        return new double[]{speed, angle};
    }

    static double[] residual(Ball b, double d, double speed, double angle) {
        Shot s = fly(b, d, speed, angle, 0, null);
        if (!s.crossed) return new double[]{1, 1};
        return new double[]{s.s, s.entryDeg - WINDOW_LEAN_DEG};
    }

    /** Plus-or-minus percentage of speed error a launch allows, with the angle exactly right. */
    static double speedTolerancePct(Ball b, double d, double speed, double angle) {
        double lo = speed, hi = speed;
        if (!fly(b, d, speed, angle, 0, null).scores()) return 0;
        while (lo > 1 && fly(b, d, lo - 0.005, angle, 0, null).scores()) lo -= 0.005;
        while (hi < 20 && fly(b, d, hi + 0.005, angle, 0, null).scores()) hi += 0.005;
        return 100 * Math.min(speed - lo, hi - speed) / speed;
    }

    /** Plus-or-minus degrees of launch angle error allowed, with the speed exactly right. */
    static double angleTolerance(Ball b, double d, double speed, double angle) {
        double lo = angle, hi = angle;
        if (!fly(b, d, speed, angle, 0, null).scores()) return 0;
        while (lo > 5 && fly(b, d, speed, lo - 0.05, 0, null).scores()) lo -= 0.05;
        while (hi < 89.9 && fly(b, d, speed, hi + 0.05, 0, null).scores()) hi += 0.05;
        return Math.min(angle - lo, hi - angle);
    }

    /** Sideways room at the window, in metres either side, for a shot at this speed and angle. */
    static double lateralRoom(Ball b, double d, double speed, double angle) {
        double lo = 0, hi = WINDOW_WIDTH_M / 2;
        if (!fly(b, d, speed, angle, 0, null).scores()) return 0;
        for (int i = 0; i < 30; i++) {
            double mid = (lo + hi) / 2;
            if (fly(b, d, speed, angle, mid, null).scores()) lo = mid; else hi = mid;
        }
        return lo;
    }

    /** Wheel RPM for a ball speed: the ball rolls on the hood, so its centre moves at half the wheel's surface speed. */
    static double wheelRpm(double ballSpeed, double efficiency) {
        double surface = 2 * ballSpeed / efficiency;
        return surface / (2 * Math.PI * WHEEL_RADIUS_M) * 60;
    }

    // =====================================================================================
    //  Hit probability
    //
    //  No shooter is perfect. Every shot leaves a little faster or slower, and a little higher or
    //  lower, than we asked for. We describe those random errors with a "typical size" (one
    //  standard deviation) and ask: if we aim at this speed and angle, what fraction of shots score?
    //  The best launch is the one with the highest fraction, not the one that looks best on paper.
    // =====================================================================================

    static final double A_MIN = 30, A_MAX = 80, A_STEP = 0.25;  // launch angles we consider (degrees)
    static final double V_MIN = 4, V_MAX = 12, V_STEP = 0.01;   // ball speeds we consider (m/s)
    static final int NA = (int) Math.round((A_MAX - A_MIN) / A_STEP) + 1;
    static final int NV = (int) Math.round((V_MAX - V_MIN) / V_STEP) + 1;

    static double angleAt(int i) { return A_MIN + i * A_STEP; }
    static double speedAt(int j) { return V_MIN + j * V_STEP; }

    static class Errors {
        final String name; final double speedPct, angleDeg;
        Errors(String name, double speedPct, double angleDeg) { this.name = name; this.speedPct = speedPct; this.angleDeg = angleDeg; }
    }

    // Our best guess for a well-built single-wheel shooter with the PID from SHOOTER-PID.md.
    static final Errors TYPICAL = new Errors("typical: speed 1.5%, angle 1.5°", 1.5, 1.5);

    /** true where a perfect shot at that (angle, speed) scores. */
    static boolean[][] scoreGrid(Ball b, double d) {
        boolean[][] ok = new boolean[NA][NV];
        for (int i = 0; i < NA; i++)
            for (int j = 0; j < NV; j++) ok[i][j] = fly(b, d, speedAt(j), angleAt(i), 0, null).scores();
        return ok;
    }

    /** Fraction of shots that score when aiming at each (angle, speed), given random errors. */
    static double[][] hitProbability(boolean[][] ok, Errors e) {
        // Blur the yes/no map with the error spread: first along speed, then along angle.
        double[][] p1 = new double[NA][NV];
        for (int i = 0; i < NA; i++) {
            for (int j = 0; j < NV; j++) {
                double sigma = e.speedPct / 100 * speedAt(j) / V_STEP;
                int reach = (int) Math.ceil(3 * sigma);
                double sum = 0, wsum = 0;
                for (int k = j - reach; k <= j + reach; k++) {
                    double w = Math.exp(-0.5 * (k - j) * (k - j) / (sigma * sigma));
                    wsum += w;
                    if (k >= 0 && k < NV && ok[i][k]) sum += w;
                }
                p1[i][j] = sum / wsum;
            }
        }
        double[][] p = new double[NA][NV];
        double sigma = e.angleDeg / A_STEP;
        int reach = (int) Math.ceil(3 * sigma);
        for (int i = 0; i < NA; i++) {
            for (int j = 0; j < NV; j++) {
                double sum = 0, wsum = 0;
                for (int k = i - reach; k <= i + reach; k++) {
                    double w = Math.exp(-0.5 * (k - i) * (k - i) / (sigma * sigma));
                    wsum += w;
                    if (k >= 0 && k < NA) sum += w * p1[k][j];
                }
                p[i][j] = sum / wsum;
            }
        }
        return p;
    }

    /** The (angle index, speed index) with the highest hit probability, optionally on one angle row. */
    static int[] best(double[][] p, int onlyRow) {
        int bi = 0, bj = 0;
        for (int i = 0; i < NA; i++) {
            if (onlyRow >= 0 && i != onlyRow) continue;
            for (int j = 0; j < NV; j++) if (p[i][j] > p[bi][bj] || (onlyRow >= 0 && bi != onlyRow)) { bi = i; bj = j; }
        }
        return new int[]{bi, bj};
    }

    static double lookup(double[][] p, double angle, double speed) {
        int i = (int) Math.round((angle - A_MIN) / A_STEP), j = (int) Math.round((speed - V_MIN) / V_STEP);
        if (i < 0 || i >= NA || j < 0 || j >= NV) return 0;
        return p[i][j];
    }

    // =====================================================================================
    //  Report
    // =====================================================================================

    static final double[] DISTANCES = {1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0};

    public static void main(String[] args) throws Exception {
        String outDir = args.length > 0 ? args[0] : null;
        if (outDir != null) new File(outDir).mkdirs();

        Ball base = new Ball("baseline (35 g, Cd 0.5, no spin)");

        System.out.println("A. Launch for a straight-on entry through the window centre");
        System.out.println("   dist(m)  no air: speed(m/s) angle   with drag: speed(m/s) angle  apex above window(m)  flight(s)  wheelRPM(no slip)");
        for (double d : DISTANCES) {
            double tan = Math.tan(Math.toRadians(WINDOW_LEAN_DEG));
            double t = Math.sqrt(2 * (WINDOW_HEIGHT_M + d * tan) / G);
            double vx = d / t, vz = G * t - d * tan / t;
            double[] n = normalEntry(base, d);
            List<double[]> path = new ArrayList<>();
            fly(base, d, n[0], n[1], 0, path);
            double apex = 0;
            for (double[] p : path) apex = Math.max(apex, p[1]);
            System.out.printf("   %5.1f   %13.2f %6.1f   %16.2f %6.1f   %14.2f   %12.2f   %10.0f%n",
                    d, Math.hypot(vx, vz), Math.toDegrees(Math.atan2(vz, vx)), n[0], n[1],
                    apex - WINDOW_HEIGHT_M, path.size() * 0.001, wheelRpm(n[0], 1.0));
        }

        // Score maps for every distance we need (the slow part: about 160,000 flights each).
        double[] allD = new double[13];
        for (int k = 0; k < allD.length; k++) allD[k] = 1.0 + 0.25 * k;
        boolean[][][] grids = new boolean[allD.length][][];
        for (int k = 0; k < allD.length; k++) grids[k] = scoreGrid(base, allD[k]);

        System.out.println("\nB. Straight-on launch against the launch with the best hit rate (" + TYPICAL.name + ")");
        System.out.println("   Tolerances: how far ONE error can go (the other error at zero) before the ball misses");
        System.out.println("   dist | straight on: angle speed  hit%  spdTol angTol side | best: angle speed  hit%  spdTol angTol side entry | RPM no slip");
        for (double d : DISTANCES) {
            boolean[][] ok = grids[indexOf(allD, d)];
            double[][] p = hitProbability(ok, TYPICAL);
            double[] n = normalEntry(base, d);
            int[] b = best(p, -1);
            double ba = angleAt(b[0]), bv = speedAt(b[1]);
            System.out.printf("   %3.1f |  %10.1f° %5.2f %4.0f%%  ±%4.1f%% ±%4.1f° %3.0fcm | %5.1f° %5.2f %4.0f%%  ±%4.1f%% ±%4.1f° %3.0fcm  %3.0f° | %5.0f%n",
                    d, n[1], n[0], 100 * lookup(p, n[1], n[0]),
                    speedTolerancePct(base, d, n[0], n[1]), angleTolerance(base, d, n[0], n[1]), 100 * lateralRoom(base, d, n[0], n[1]),
                    ba, bv, 100 * p[b[0]][b[1]],
                    speedTolerancePct(base, d, bv, ba), angleTolerance(base, d, bv, ba), 100 * lateralRoom(base, d, bv, ba),
                    fly(base, d, bv, ba, 0, null).entryDeg, wheelRpm(bv, 1.0));
        }

        System.out.println("\nC. The best launch depends on which error is bigger (best angle / entry angle / hit%)");
        Errors[] budgets = {
                new Errors("speed 0.5%, angle 1.5° (very steady speed)", 0.5, 1.5),
                TYPICAL,
                new Errors("speed 3%, angle 1.5° (no PID, burst sags)", 3.0, 1.5),
                new Errors("speed 1.5%, angle 0.5° (very steady angle)", 1.5, 0.5)};
        for (Errors e : budgets) {
            System.out.printf("   %-44s", e.name);
            for (double d : new double[]{1.5, 2.5, 3.5}) {
                double[][] p = hitProbability(grids[indexOf(allD, d)], e);
                int[] b = best(p, -1);
                double entry = fly(base, d, speedAt(b[1]), angleAt(b[0]), 0, null).entryDeg;
                System.out.printf("  %.1fm: %4.1f° /%3.0f° /%4.0f%%", d, angleAt(b[0]), entry, 100 * p[b[0]][b[1]]);
            }
            System.out.println();
        }

        System.out.println("\nD. One fixed hood angle for every distance (" + TYPICAL.name + ")");
        double[][] bestHitAtAngle = new double[allD.length][NA]; // best over speed, for each distance and angle
        double[][] bestSpeedAtAngle = new double[allD.length][NA];
        for (int k = 0; k < allD.length; k++) {
            double[][] p = hitProbability(grids[k], TYPICAL);
            for (int i = 0; i < NA; i++) {
                int[] b = best(p, i);
                bestHitAtAngle[k][i] = p[b[0]][b[1]];
                bestSpeedAtAngle[k][i] = speedAt(b[1]);
            }
        }
        double[][] ranges = {{1.0, 3.0}, {1.5, 3.0}, {1.5, 4.0}, {1.0, 4.0}};
        int chosenRow = 0;
        for (double[] r : ranges) {
            int bestRow = 0; double bestWorst = -1;
            for (int i = 0; i < NA; i++) {
                double worst = 1;
                for (int k = 0; k < allD.length; k++)
                    if (allD[k] >= r[0] - 1e-9 && allD[k] <= r[1] + 1e-9) worst = Math.min(worst, bestHitAtAngle[k][i]);
                if (worst > bestWorst) { bestWorst = worst; bestRow = i; }
            }
            if (r[0] == 1.5 && r[1] == 3.0) chosenRow = bestRow;
            System.out.printf("   %.1f-%.1f m: best fixed angle %.1f°, worst hit rate in that range %.0f%%%n", r[0], r[1], angleAt(bestRow), 100 * bestWorst);
        }
        double fixedAngle = angleAt(chosenRow);
        System.out.printf("%n   Speed table for a fixed %.1f° hood%n", fixedAngle);
        System.out.println("   dist(m)  speed(m/s)  hit%  (best possible)  entry  wheelRPM(no slip)  wheelRPM(80% efficiency)");
        for (int k = 0; k < allD.length; k++) {
            double v = bestSpeedAtAngle[k][chosenRow];
            double bestAny = 0;
            for (int i = 0; i < NA; i++) bestAny = Math.max(bestAny, bestHitAtAngle[k][i]);
            Shot s = fly(base, allD[k], v, fixedAngle, 0, null);
            if (bestHitAtAngle[k][chosenRow] < 0.05) {
                System.out.printf("   %5.2f   cannot score at this angle      (%3.0f%%)%n", allD[k], 100 * bestAny);
                continue;
            }
            System.out.printf("   %5.2f   %8.2f   %4.0f%%   (%3.0f%%)       %4.0f°  %10.0f  %16.0f%n",
                    allD[k], v, 100 * bestHitAtAngle[k][chosenRow], 100 * bestAny,
                    s.crossed ? s.entryDeg : Double.NaN, wheelRpm(v, 1.0), wheelRpm(v, 0.8));
        }

        System.out.println("\nE. How much the answers depend on the estimates (best angle / speed / hit%, " + TYPICAL.name + ")");
        Ball lessDrag = base.copy("less drag (50 g, or Cd 0.35)"); lessDrag.kg = 0.050;
        Ball moreDrag = base.copy("more drag (25 g, or Cd 0.7)"); moreDrag.kg = 0.025;
        Ball noAir = base.copy("no air at all"); noAir.cd = 0;
        Ball spin1 = base.copy("backspin lift CL 0.1"); spin1.cl = 0.1;
        Ball spin2 = base.copy("backspin lift CL 0.2"); spin2.cl = 0.2;
        Ball apexDown = base.copy("triangle point down"); apexDown.apexUp = false;
        for (Ball b : new Ball[]{base, noAir, lessDrag, moreDrag, spin1, spin2, apexDown}) {
            System.out.printf("   %-34s", b.name);
            for (double d : new double[]{1.5, 2.5, 3.5}) {
                boolean[][] ok = (b == base) ? grids[indexOf(allD, d)] : scoreGrid(b, d);
                double[][] p = hitProbability(ok, TYPICAL);
                int[] bb = best(p, -1);
                System.out.printf("  %.1fm: %4.1f° %4.2fm/s %3.0f%%", d, angleAt(bb[0]), speedAt(bb[1]), 100 * p[bb[0]][bb[1]]);
            }
            System.out.println();
        }

        if (outDir == null) return;

        // Example flights at 2.5 m: straight on, the best-hit-rate launch, and a flat and a steep shot.
        try (PrintWriter w = new PrintWriter(outDir + "/paths.csv")) {
            w.println("name,x,z");
            double d = 2.5;
            double[] n = normalEntry(base, d);
            writePath(w, "normal", base, d, n[0], n[1]);
            int[] b = best(hitProbability(grids[indexOf(allD, d)], TYPICAL), -1);
            writePath(w, "best", base, d, speedAt(b[1]), angleAt(b[0]));
            for (double a : new double[]{55, 72}) writePath(w, "a" + (int) a, base, d, speedThroughCentre(base, d, a), a);
        }
        // Score map and hit probability (launch angle x speed) for three distances, thinned out.
        try (PrintWriter w = new PrintWriter(outDir + "/map.csv")) {
            w.println("dist,angle,speed,scores,hit");
            for (double d : new double[]{1.5, 2.5, 3.5}) {
                boolean[][] ok = grids[indexOf(allD, d)];
                double[][] p = hitProbability(ok, TYPICAL);
                for (int i = 0; i < NA; i += 1)
                    for (int j = 0; j < NV; j += 2)
                        w.printf("%.1f,%.2f,%.2f,%d,%.4f%n", d, angleAt(i), speedAt(j), ok[i][j] ? 1 : 0, p[i][j]);
            }
        }
        try (PrintWriter w = new PrintWriter(outDir + "/launch.csv")) {
            w.println("dist,normal_angle,normal_speed,best_angle,best_speed,best_hit,normal_hit");
            for (int k = 0; k < allD.length; k++) {
                double[][] p = hitProbability(grids[k], TYPICAL);
                double[] n = normalEntry(base, allD[k]);
                int[] b = best(p, -1);
                w.printf("%.2f,%.2f,%.3f,%.2f,%.3f,%.4f,%.4f%n", allD[k], n[1], n[0], angleAt(b[0]), speedAt(b[1]), p[b[0]][b[1]], lookup(p, n[1], n[0]));
            }
        }
        try (PrintWriter w = new PrintWriter(outDir + "/fixed.csv")) {
            w.println("dist,angle,hit,speed");
            for (int k = 0; k < allD.length; k++)
                for (int i = 0; i < NA; i++)
                    w.printf("%.2f,%.2f,%.4f,%.3f%n", allD[k], angleAt(i), bestHitAtAngle[k][i], bestSpeedAtAngle[k][i]);
        }
    }

    static int indexOf(double[] a, double v) {
        for (int i = 0; i < a.length; i++) if (Math.abs(a[i] - v) < 1e-9) return i;
        throw new IllegalArgumentException("no distance " + v);
    }

    static double speedThroughCentre(Ball b, double d, double angle) {
        double lo = 3, hi = 14;
        for (int i = 0; i < 40; i++) {
            double mid = (lo + hi) / 2;
            Shot s = fly(b, d, mid, angle, 0, null);
            // Too slow: crosses below the centre (or never gets there).
            if (!s.reached || s.s < 0) lo = mid; else hi = mid;
        }
        return (lo + hi) / 2;
    }

    static void writePath(PrintWriter w, String name, Ball b, double d, double v, double a) {
        List<double[]> path = new ArrayList<>();
        fly(b, d, v, a, 0, path);
        for (double[] p : path) w.printf("%s,%.4f,%.4f%n", name, p[0], p[1]);
    }
}
