import org.firstinspires.ftc.teamcode.hw.VelocityPidController;

import java.io.PrintWriter;
import java.util.Random;

/**
 * Laptop simulation of the shooter. It runs the REAL VelocityPidController against a physics model
 * of the motor, flywheel, Through Bore Encoder, OctoQuad velocity register and a robot loop that
 * takes 20-40 ms.
 *
 * It does not replace testing on the robot. The motor numbers come from the goBILDA data sheet, but
 * the ball mass, the inertia of the Rhino wheel and the energy a shot takes are estimates. Use it to
 * compare ideas, and trust the robot for the final numbers.
 *
 * Run it from the docs/shooter_sim folder (needs a JDK, not the Android tools):
 *   javac -d out -sourcepath ../../FtcRobotController/TeamCode/src/main/java ShooterSim.java
 *   java -cp out ShooterSim            prints the tables used in the document
 *   java -cp out ShooterSim out/data   also writes CSV traces for the plots
 */
public class ShooterSim {

    // Motor: goBILDA 5203 6000 RPM data sheet (12 V, 9.2 A stall, 1.47 kg-cm stall, 0.25 A free)
    static final double STALL_AMPS = 9.2, FREE_AMPS = 0.25, STALL_NM = 1.47 * 0.0980665;
    static final double FREE_RAD_S = 6000 * 2 * Math.PI / 60, RATED_VOLTS = 12;
    static final double OHMS = RATED_VOLTS / STALL_AMPS;
    static final double KT = STALL_NM / STALL_AMPS;
    static final double KE = (RATED_VOLTS - FREE_AMPS * OHMS) / FREE_RAD_S;
    static final double FRICTION_NM = KT * FREE_AMPS;

    // Rotating parts: flywheel 1651 g-cm^2 (goBILDA), Rhino wheel 85 g estimated as a thick ring,
    // plus an estimate for hub, shaft and motor rotor.
    static final double WHEEL_RADIUS_M = 0.036;
    static final double FLYWHEEL_J = 1651e-7;
    static final double RHINO_J = 0.5 * 0.085 * (0.036 * 0.036 + 0.020 * 0.020);
    static final double OTHER_J = 1e-5;

    static final double SIM_STEP_S = 0.0005;
    static final double COMMAND_DELAY_S = 0.005; // time for a setPower() to reach the motor

    static class Config {
        double kS = 0.03, kV = 1.0 / 6000, kP = 0.002, kI = 0.0006, kD = 0;
        boolean loopVariable = true;
        double windowMs = 20;
        double countsPerRev = 8192;     // REV Through Bore Encoder. The motor's own encoder gives 28
        double angleErrorDeg = 0.5;     // the Through Bore is factory calibrated to +/- 0.5 degrees
        double targetRpm = 4500;
        double feedGapMs = 300;
        double ballKg = 0.035;
        double inertia = FLYWHEEL_J + RHINO_J + OTHER_J;
        double batteryVolts = 12.5;
        int balls = 4;
        double firstBallS = 3.0;
        int motors = 1;
        boolean waitUntilReady = false;
        double readyToleranceRpm = 100;
    }

    static class Result {
        double spinUpS = Double.NaN;
        double[] ballRpm;
        double maxDip;
        double lastBallS;
        double recoveryAfterFirstMs = Double.NaN;
        double overshootRpm;
        double rmsErrorRpm;
        double powerJitter;
    }

    static Result run(Config c, PrintWriter csv) {
        Random random = new Random(42);
        VelocityPidController pid = new VelocityPidController();
        pid.setGains(c.kS, c.kV, c.kP, c.kI, c.kD);
        pid.setLimits(300, 0.15, 0.3, 12);

        double omega = 0, angle = 0;
        double t = 0, nextLoop = 0, lastLoop = 0;
        long step = 0;
        long windowSteps = Math.round(c.windowMs / 1000 / SIM_STEP_S);
        long windowStartCounts = 0;
        double registerRpm = 0;
        double appliedPower = 0, pendingPower = 0, pendingAt = Double.MAX_VALUE;

        Result r = new Result();
        r.ballRpm = new double[c.balls];
        int ball = 0;
        double nextBallT = c.waitUntilReady ? Double.MAX_VALUE : c.firstBallS;
        double contactUntil = 0, contactTorque = 0;
        double firstBallT = -1;
        double readySince = -1;
        boolean armed = !c.waitUntilReady;
        double power = 0, sumSq = 0, sumDiffSq = 0, lastPower = 0;
        int settledSamples = 0;
        double end = c.firstBallS + 4.0;
        boolean recovered = false;
        double minAfterFirst = Double.MAX_VALUE;

        while (t < end) {
            // ---- robot loop: read the register, run the controller, send the command ----
            if (t >= nextLoop) {
                double dt = t - lastLoop;
                lastLoop = t;
                power = pid.update(c.targetRpm, registerRpm, dt, c.batteryVolts);
                pendingPower = power;
                pendingAt = t + COMMAND_DELAY_S;
                double loopS = c.loopVariable ? 0.020 + 0.020 * random.nextDouble() : 0.030;
                nextLoop = t + loopS;

                if (t > 2.5 && t < c.firstBallS - 0.05) {
                    sumDiffSq += (power - lastPower) * (power - lastPower);
                    sumSq += (c.targetRpm - registerRpm) * (c.targetRpm - registerRpm);
                    settledSamples++;
                }
                lastPower = power;

                if (Math.abs(c.targetRpm - registerRpm) < c.readyToleranceRpm) {
                    if (readySince < 0) readySince = t;
                } else {
                    readySince = -1;
                }
                // A "ready gate" feeds the next ball only when the wheel is back on speed.
                if (c.waitUntilReady && !armed && t >= c.firstBallS && ball < c.balls
                        && t >= contactUntil + c.feedGapMs / 1000 && readySince >= 0 && t - readySince > 0.05) {
                    nextBallT = t + 0.03;
                    armed = true;
                }
                if (csv != null) {
                    csv.printf("%.3f,%.1f,%.1f,%.3f%n", t, omega * 60 / (2 * Math.PI), registerRpm, power);
                }
            }
            if (t >= pendingAt) {
                appliedPower = pendingPower;
                pendingAt = Double.MAX_VALUE;
            }

            // ---- a ball enters the wheel ----
            if (t >= nextBallT && ball < c.balls) {
                double rpmNow = omega * 60 / (2 * Math.PI);
                r.ballRpm[ball] = rpmNow;
                // The ball rolls along the hood, so its centre moves at half the wheel surface speed.
                // Ball energy = 5/6 m v^2 (hollow ball, moving + spinning). Slip wastes about as much
                // again, so the wheel gives up twice the ball's energy over a 40 ms contact.
                double v = omega * WHEEL_RADIUS_M / 2;
                double energy = 2 * (5.0 / 6.0) * c.ballKg * v * v;
                contactTorque = energy / (omega * 0.040);
                contactUntil = t + 0.040;
                if (firstBallT < 0) firstBallT = t;
                r.lastBallS = t - firstBallT;
                ball++;
                if (c.waitUntilReady) {
                    armed = false;
                    nextBallT = Double.MAX_VALUE;
                } else {
                    nextBallT = t + c.feedGapMs / 1000;
                }
                readySince = -1;
            }

            // ---- physics step. In coast mode the driver cannot make the current go negative ----
            double amps = Math.max(0, (appliedPower * c.batteryVolts - KE * omega) / OHMS);
            double torque = c.motors * (KT * amps - FRICTION_NM);
            if (t < contactUntil) torque -= contactTorque;
            omega = Math.max(0, omega + torque / c.inertia * SIM_STEP_S);
            angle += omega * SIM_STEP_S;
            t += SIM_STEP_S;

            // ---- OctoQuad: counts whole encoder steps, publishes a new value each window ----
            // A real 16-bit register wraps around if a window holds more than 32767 counts.
            step++;
            if (step % windowSteps == 0) {
                double seenAngle = angle + Math.toRadians(c.angleErrorDeg) * Math.sin(angle);
                long counts = (long) Math.floor(seenAngle / (2 * Math.PI) * c.countsPerRev);
                short register = (short) (counts - windowStartCounts);
                windowStartCounts = counts;
                registerRpm = register / c.countsPerRev / (c.windowMs / 1000) * 60;
            }

            // ---- statistics ----
            double rpm = omega * 60 / (2 * Math.PI);
            if (Double.isNaN(r.spinUpS) && rpm >= c.targetRpm - c.readyToleranceRpm) r.spinUpS = t;
            if (t < c.firstBallS - 0.05) r.overshootRpm = Math.max(r.overshootRpm, rpm - c.targetRpm);
            if (firstBallT >= 0) {
                minAfterFirst = Math.min(minAfterFirst, rpm);
                r.maxDip = Math.max(r.maxDip, c.targetRpm - rpm);
            }
            if (c.balls == 1 && firstBallT >= 0 && t > contactUntil && !recovered
                    && rpm >= c.targetRpm - c.readyToleranceRpm) {
                r.recoveryAfterFirstMs = (t - firstBallT) * 1000;
                recovered = true;
            }
        }
        r.rmsErrorRpm = settledSamples > 0 ? Math.sqrt(sumSq / settledSamples) : 0;
        r.powerJitter = settledSamples > 0 ? Math.sqrt(sumDiffSq / settledSamples) : 0;
        return r;
    }

    /** How much of the wheel's speed one ball takes, as a percentage. */
    static double singleDipPercent(Config base) {
        Config c = copy(base);
        c.balls = 1;
        Result r = run(c, null);
        return 100 * r.maxDip / c.targetRpm;
    }

    static Config copy(Config b) {
        Config c = new Config();
        c.kS = b.kS; c.kV = b.kV; c.kP = b.kP; c.kI = b.kI; c.kD = b.kD;
        c.countsPerRev = b.countsPerRev; c.angleErrorDeg = b.angleErrorDeg;
        c.loopVariable = b.loopVariable; c.windowMs = b.windowMs; c.targetRpm = b.targetRpm;
        c.feedGapMs = b.feedGapMs; c.ballKg = b.ballKg; c.inertia = b.inertia; c.batteryVolts = b.batteryVolts;
        c.balls = b.balls; c.firstBallS = b.firstBallS; c.waitUntilReady = b.waitUntilReady;
        c.readyToleranceRpm = b.readyToleranceRpm; c.motors = b.motors;
        return c;
    }

    static String balls(Result r, double target) {
        StringBuilder s = new StringBuilder();
        for (double rpm : r.ballRpm) s.append(String.format("%6.0f", rpm - target));
        return s.toString();
    }

    public static void main(String[] args) throws Exception {
        Config base = new Config();
        System.out.printf("J total = %.1f g-cm^2 (flywheel %.0f, Rhino %.0f, other %.0f)%n",
                base.inertia * 1e7, FLYWHEEL_J * 1e7, RHINO_J * 1e7, OTHER_J * 1e7);
        System.out.printf("motor: R=%.2f ohm, Kt=%.4f Nm/A, Ke=%.4f V/(rad/s), time constant=%.2f s%n",
                OHMS, KT, KE, base.inertia * OHMS / (KT * KE));
        System.out.printf("one ball takes %.1f%% of wheel speed (ball %.0f g)%n%n", singleDipPercent(base), base.ballKg * 1000);

        System.out.println("A. Resolution and the 16-bit limit (Through Bore, 8192 counts per rev)");
        for (int ms : new int[]{5, 10, 15, 20, 25, 30, 40, 50}) {
            System.out.printf("   %3d ms: %5.2f RPM per count, %6.0f counts at 4500 RPM, wraps above %6.0f RPM%n",
                    ms, 60000.0 / (8192 * ms), 4500 * 8192 * ms / 60000.0, 32767 * 60000.0 / (8192 * ms));
        }

        System.out.println("\nB. OctoQuad window at 4500 RPM (settled, before any ball)");
        System.out.println("   window  rmsErr(RPM)  powerJitter  ballError(RPM) 1..4");
        for (int ms : new int[]{5, 10, 15, 20, 25, 30}) {
            Config c = copy(base);
            c.windowMs = ms;
            Result r = run(c, null);
            System.out.printf("   %4d    %8.1f     %8.4f   %s%n", ms, r.rmsErrorRpm, r.powerJitter, balls(r, c.targetRpm));
        }

        System.out.println("\nB2. Motor encoder (28 counts, 40 ms) vs Through Bore (8192 counts), 4500 RPM, 250 ms apart");
        Object[][] encoders = {
                {"motor encoder, kP .0015", 28.0, 40.0, 0.0, 0.0015, 0.0},
                {"Through Bore, kP .0015", 8192.0, 20.0, 0.5, 0.0015, 0.0},
                {"Through Bore, kP .002", 8192.0, 20.0, 0.5, 0.002, 0.0},
                {"Through Bore, kP .006", 8192.0, 20.0, 0.5, 0.006, 0.0},
                {"Through Bore, kP .012", 8192.0, 20.0, 0.5, 0.012, 0.0},
                {"Through Bore, kP .002 kD 1e-5", 8192.0, 20.0, 0.5, 0.002, 1e-5},
                {"Through Bore, kP .002 kD 5e-5", 8192.0, 20.0, 0.5, 0.002, 5e-5},
                {"motor encoder, kP .0015 kD 5e-5", 28.0, 40.0, 0.0, 0.0015, 5e-5},
        };
        for (Object[] row : encoders) {
            Config c = copy(base);
            c.countsPerRev = (double) row[1]; c.windowMs = (double) row[2]; c.angleErrorDeg = (double) row[3];
            c.kP = (double) row[4]; c.kD = (double) row[5]; c.feedGapMs = 250;
            Result r = run(c, null);
            System.out.printf("   %-33s rms %5.1f  jitter %.4f  over %4.0f  %s%n",
                    row[0], r.rmsErrorRpm, r.powerJitter, r.overshootRpm, balls(r, c.targetRpm));
        }

        System.out.println("\nB3. What the old 40 ms window would do with the Through Bore on a fresh battery");
        for (double volts : new double[]{12.5, 13.5}) {
            Config c = copy(base);
            c.windowMs = 40; c.targetRpm = 6400; c.batteryVolts = volts; c.balls = 0;
            Result r = run(c, null);
            System.out.printf("   %.1f V, target 6400 RPM: rms error %8.1f RPM%n", volts, r.rmsErrorRpm);
        }

        System.out.println("\nC. Controller comparison, 4 balls, 300 ms apart, 4500 RPM. Error when each ball arrives");
        Object[][] controllers = {
                {"open loop (kV only)", 0.03, 1.0 / 6000, 0.0, 0.0},
                {"P only (kP .0005)", 0.0, 0.0, 0.0005, 0.0},
                {"FF + P (kP .002)", 0.03, 1.0 / 6000, 0.002, 0.0},
                {"FF + PI (kP .0015)", 0.03, 1.0 / 6000, 0.0015, 0.0006},
                {"FF + PI (kP .002)", 0.03, 1.0 / 6000, 0.002, 0.0006},
                {"FF + PI (kP .004)", 0.03, 1.0 / 6000, 0.004, 0.0006},
                {"FF + PI (kP .012)", 0.03, 1.0 / 6000, 0.012, 0.0006},
        };
        for (Object[] row : controllers) {
            Config c = copy(base);
            c.kS = (double) row[1]; c.kV = (double) row[2]; c.kP = (double) row[3]; c.kI = (double) row[4];
            Result r = run(c, null);
            System.out.printf("   %-22s spinUp %5.2fs  over %5.0f  jitter %.3f  %s%n",
                    row[0], r.spinUpS, r.overshootRpm, r.powerJitter, balls(r, c.targetRpm));
        }

        System.out.println("\nC2. kP sweep, 5000 RPM, 250 ms apart, FF + PI");
        for (double kP : new double[]{0.0005, 0.001, 0.0015, 0.002, 0.003, 0.004, 0.006, 0.008, 0.012}) {
            Config c = copy(base);
            c.kP = kP; c.targetRpm = 5000; c.feedGapMs = 250;
            Result r = run(c, null);
            System.out.printf("   kP %.4f  jitter %.3f  over %4.0f  %s%n", kP, r.powerJitter, r.overshootRpm, balls(r, c.targetRpm));
        }

        System.out.println("\nD. kV wrong by 10% (kV * 0.9): does the integral fix it?  rms error while holding");
        for (double kI : new double[]{0, 0.0006}) {
            Config c = copy(base);
            c.kV = 0.9 / 6000; c.kI = kI;
            Result r = run(c, null);
            System.out.printf("   kI=%.4f  rmsErr %6.1f RPM%n", kI, r.rmsErrorRpm);
        }

        System.out.println("\nE. Battery 11.5 V instead of 12.5 V, default gains");
        for (double v : new double[]{12.5, 11.5}) {
            Config c = copy(base);
            c.batteryVolts = v;
            Result r = run(c, null);
            System.out.printf("   %.1f V  rmsErr %5.1f  %s%n", v, r.rmsErrorRpm, balls(r, c.targetRpm));
        }

        System.out.println("\nF. Recovery after ONE ball vs. target speed (time to return within 100 RPM)");
        for (double rpm : new double[]{3000, 3500, 4000, 4500, 5000, 5500}) {
            Config c = copy(base);
            c.targetRpm = rpm; c.balls = 1;
            Result r = run(c, null);
            System.out.printf("   %4.0f RPM  dip %4.0f  recovery %5.0f ms%n", rpm, r.maxDip, r.recoveryAfterFirstMs);
        }

        System.out.println("\nG. Burst of 4 vs. feed spacing and target. Error (RPM) as each ball arrives");
        for (double rpm : new double[]{3500, 4500, 5000, 5500}) {
            for (double gap : new double[]{150, 250, 350, 500}) {
                Config c = copy(base);
                c.targetRpm = rpm; c.feedGapMs = gap;
                Result r = run(c, null);
                System.out.printf("   %4.0f RPM gap %3.0f  burst %4.2fs  %s%n", rpm, gap, r.lastBallS, balls(r, rpm));
            }
        }

        System.out.println("\nH. Ready gate (feed next ball when within tolerance)");
        for (double rpm : new double[]{3500, 4500, 5000, 5500}) {
            Config c = copy(base);
            c.targetRpm = rpm; c.waitUntilReady = true; c.feedGapMs = 50; c.readyToleranceRpm = 150;
            Result r = run(c, null);
            System.out.printf("   %4.0f RPM  burst %4.2fs  %s%n", rpm, r.lastBallS, balls(r, rpm));
        }

        System.out.println("\nI. Mechanical options at 4500 RPM, 250 ms apart");
        Object[][] mech = {
                {"as built", 1.0},
                {"second steel flywheel", (FLYWHEEL_J * 2 + RHINO_J + OTHER_J) / (FLYWHEEL_J + RHINO_J + OTHER_J)},
        };
        for (double rpm : new double[]{4500, 5500}) {
            Config c = copy(base);
            c.feedGapMs = 250; c.motors = 2; c.targetRpm = rpm;
            Result r = run(c, null);
            System.out.printf("   second motor at %4.0f RPM  spinUp %4.2fs  %s%n", rpm, r.spinUpS, balls(r, rpm));
        }
        for (Object[] m : mech) {
            Config c = copy(base);
            c.feedGapMs = 250; c.inertia *= (double) m[1];
            Result r = run(c, null);
            System.out.printf("   %-24s spinUp %4.2fs  %s%n", m[0], r.spinUpS, balls(r, c.targetRpm));
        }

        System.out.println("\nJ. Sensitivity to ball mass, 4500 RPM, 250 ms apart");
        for (double kg : new double[]{0.020, 0.035, 0.050}) {
            Config c = copy(base);
            c.feedGapMs = 250; c.ballKg = kg;
            Result r = run(c, null);
            System.out.printf("   %2.0f g   %s%n", kg * 1000, balls(r, c.targetRpm));
        }

        if (args.length > 0) {
            String dir = args[0];
            new java.io.File(dir).mkdirs();
            writeTrace(dir + "/open_loop.csv", 0.03, 1.0 / 6000, 0, 0, 4500, 250, false);
            writeTrace(dir + "/pid.csv", 0.03, 1.0 / 6000, base.kP, 0.0006, 4500, 250, false);
            writeTrace(dir + "/ready_gate.csv", 0.03, 1.0 / 6000, base.kP, 0.0006, 4500, 50, true);
            writeTrace(dir + "/high_rpm.csv", 0.03, 1.0 / 6000, base.kP, 0.0006, 5500, 250, false);
        }
    }

    static void writeTrace(String file, double kS, double kV, double kP, double kI,
                           double rpm, double gap, boolean gate) throws Exception {
        Config c = new Config();
        c.kS = kS; c.kV = kV; c.kP = kP; c.kI = kI; c.targetRpm = rpm; c.feedGapMs = gap; c.waitUntilReady = gate;
        c.readyToleranceRpm = gate ? 150 : 100;
        try (PrintWriter w = new PrintWriter(file)) {
            w.println("t,trueRpm,measuredRpm,power");
            run(c, w);
        }
    }
}
