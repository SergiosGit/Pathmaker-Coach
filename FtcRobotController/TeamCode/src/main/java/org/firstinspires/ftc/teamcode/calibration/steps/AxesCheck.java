package org.firstinspires.ftc.teamcode.calibration.steps;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;

import org.firstinspires.ftc.teamcode.calibration.Calibration;
import org.firstinspires.ftc.teamcode.calibration.runner.Display;
import org.firstinspires.ftc.teamcode.calibration.runner.Procedure;
import org.firstinspires.ftc.teamcode.calibration.runner.TuningOpMode;

/**
 * Finds which OctoQuad port each odometry pod is on and which way it counts, from two hand pushes.
 * Nothing is powered. Catches swapped pod cables before the OctoQuad step measures offsets with
 * the wrong pods (which happened on the bay-b bot).
 */
public class AxesCheck extends Procedure {

    /** A push must move the main pod at least this far to count. */
    static final double MIN_PUSH_IN = 6;
    /** The main pod must move this many times more than any other. */
    static final double DOMINANCE = 3;

    public AxesCheck() {
        super("Odometry ports and directions",
                "Push the robot forward, then to its left. Finds the forward and strafe pod ports and directions.");
    }

    @Override
    public void run() throws InterruptedException {
        String name = Calibration.getString("octoquad.name");
        double ticksPerInch = Calibration.getDouble("octoquad.ticksPerUnit");
        int oldX = Calibration.getInt("octoquad.xPodPort"), oldY = Calibration.getInt("octoquad.yPodPort");

        long[] forward = runOpModeWithArrow("forward", new Push(name, "Push forward",
                "Push the robot straight forward about one tile (24 in), keeping its heading. Then press Finish."));
        long[] left = runOpModeWithArrow("left", new Push(name, "Push to the left",
                "Push the robot straight to ITS left about one tile, keeping its heading. Then press Finish."));

        Result r = analyse(forward, left, ticksPerInch, oldX);
        if (r.error != null) abort(r.error);

        result("octoquad.xPodPort", r.xPort);
        result("octoquad.yPodPort", r.yPort);
        result("octoquad.xPodDirection", r.xDirection);
        result("octoquad.yPodDirection", r.yDirection);
        result("Forward push", String.format("%.1f in on port %d", r.forwardInches, r.xPort));
        result("Left push", String.format("%.1f in on port %d", r.leftInches, r.yPort));
        if (r.note != null) result("Second forward pod", r.note);
        if (r.xPort != oldX || r.yPort != oldY) {
            result("Note", "The ports changed (were forward " + oldX + ", strafe " + oldY
                    + "). Run the OctoQuad step again: its offsets depend on the ports.");
        }
    }

    private long[] runOpModeWithArrow(String arrow, Push push) throws InterruptedException {
        final long[][] out = new long[1][];
        withDisplay(Display.arrow(arrow), () -> out[0] = runOpMode(push));
        return out[0];
    }

    /**
     * What two pushes say about the ports. Pure function, tested on a laptop.
     *
     * A robot may have two forward pods (three-pod odometry). Both move by about the same amount
     * in the forward push, so the forward push only has to single out the forward pods from the
     * strafe pod. The localizer uses one forward pod (heading comes from the OctoQuad's IMU):
     * the one set before if it is one of them, else the one that moved most.
     */
    static Result analyse(long[] forward, long[] left, double ticksPerInch, int preferredXPort) {
        Result r = new Result();
        int f = argmaxAbs(forward), l = argmaxAbs(left);
        r.forwardInches = Math.abs(forward[f]) / ticksPerInch;
        r.leftInches = Math.abs(left[l]) / ticksPerInch;
        if (r.forwardInches < MIN_PUSH_IN || r.leftInches < MIN_PUSH_IN) {
            r.error = String.format("A push was too short (forward %.1f in, left %.1f in; need %.0f in). "
                    + "Check that both pods touch the floor and are plugged in, then try again.",
                    r.forwardInches, r.leftInches, MIN_PUSH_IN);
            return r;
        }
        if (f == l) {
            r.error = "The same port (" + f + ") moved most in both pushes. One pod may not be plugged in "
                    + "or not touching the floor, or the pushes weren't straight.";
            return r;
        }
        // The strafe pod must stand out in the left push, and stay nearly still in the forward push.
        if (Math.abs(left[l]) < DOMINANCE * secondAbs(left, l)
                || Math.abs(forward[f]) < DOMINANCE * Math.abs(forward[l])) {
            r.error = "Two pods moved by similar amounts in one push. Push straighter, without turning, and try again.";
            return r;
        }
        // Every other port that moved about as much as the largest one is a forward pod.
        StringBuilder others = new StringBuilder();
        int x = f;
        for (int i = 0; i < forward.length; i++) {
            if (i == l || DOMINANCE * Math.abs(forward[i]) < Math.abs(forward[f])) continue;
            if (i == preferredXPort) x = i;
        }
        for (int i = 0; i < forward.length; i++) {
            if (i == l || i == x || DOMINANCE * Math.abs(forward[i]) < Math.abs(forward[f])) continue;
            others.append(others.length() == 0 ? "" : ", ").append(i);
        }
        if (others.length() > 0) {
            r.note = "Port " + others + " also measures forward. It is not used: the localizer uses one "
                    + "forward pod (port " + x + ") and the OctoQuad's IMU for heading.";
        }
        r.xPort = x;
        r.yPort = l;
        r.forwardInches = Math.abs(forward[x]) / ticksPerInch;
        // Forward motion must count up on the forward pod; motion to the left on the strafe pod.
        r.xDirection = forward[x] > 0 ? OctoQuad.EncoderDirection.FORWARD : OctoQuad.EncoderDirection.REVERSE;
        r.yDirection = left[l] > 0 ? OctoQuad.EncoderDirection.FORWARD : OctoQuad.EncoderDirection.REVERSE;
        return r;
    }

    static final class Result {
        int xPort, yPort;
        OctoQuad.EncoderDirection xDirection, yDirection;
        double forwardInches, leftInches;
        String error;
        String note;
    }

    private static int argmaxAbs(long[] a) {
        int best = 0;
        for (int i = 1; i < a.length; i++) if (Math.abs(a[i]) > Math.abs(a[best])) best = i;
        return best;
    }

    private static double secondAbs(long[] a, int skip) {
        double m = 0;
        for (int i = 0; i < a.length; i++) if (i != skip) m = Math.max(m, Math.abs(a[i]));
        return m;
    }

    /** Records how far every port moved, as if every port counted FORWARD. */
    static final class Push extends TuningOpMode<long[]> {
        private final String octoQuadName;

        Push(String octoQuadName, String title, String description) {
            super(title, description, true);
            this.octoQuadName = octoQuadName;
        }

        @Override
        protected long[] runTuningOpMode() throws InterruptedException {
            OctoQuad oq = hardwareMap.get(OctoQuad.class, octoQuadName);
            waitForStart();
            int[] start = oq.readAllEncoderData().positions.clone();
            long[] delta = new long[start.length];
            while (!isStopRequested()) {
                int[] now = oq.readAllEncoderData().positions;
                for (int i = 0; i < start.length; i++) {
                    // Undo the direction currently set on the OctoQuad, so the sign is physical.
                    int sign = oq.getSingleEncoderDirection(i) == OctoQuad.EncoderDirection.REVERSE ? -1 : 1;
                    delta[i] = sign * (long) (now[i] - start[i]);
                }
                for (int i = 0; i < 4; i++) telemetry.addData("port " + i, delta[i]);
                telemetry.update();
                sleep(20);
            }
            return delta;
        }
    }
}
