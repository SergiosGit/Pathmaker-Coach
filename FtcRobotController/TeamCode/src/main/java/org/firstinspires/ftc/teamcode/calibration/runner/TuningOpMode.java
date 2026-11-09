package org.firstinspires.ftc.teamcode.calibration.runner;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * One powered or hand-pushed run inside a step. Runs on the Calibrate OpMode's thread.
 * Same shape as Pedro AutoTune's TuningOpMode: waitForStart() waits for Start on the web page;
 * isStopRequested() becomes true on Finish (if the run can be finished by hand), Abort, or Stop
 * on the Driver Hub.
 */
public abstract class TuningOpMode<R> {

    public final String name;
    public final String description;
    /** True when the person ends the run with Finish, for example after pushing the robot. */
    public final boolean canStop;

    protected HardwareMap hardwareMap;
    protected Gamepad gamepad1, gamepad2;
    protected final PageTelemetry telemetry = new PageTelemetry();

    public TuningOpMode(String name, String description, boolean canStop) {
        this.name = name;
        this.description = description;
        this.canStop = canStop;
    }

    protected abstract R runTuningOpMode() throws InterruptedException;

    final R execute(CalibrationSession.Host host) throws InterruptedException {
        this.hardwareMap = host.hardwareMap();
        this.gamepad1 = host.gamepad1();
        this.gamepad2 = host.gamepad2();
        return runTuningOpMode();
    }

    /** Like LinearOpMode.waitForStart(), declares no exception; Stop or Abort ends the step. */
    protected final void waitForStart() {
        try {
            CalibrationSession.get().awaitStart(this);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new CalibrationAbort("the OpMode was stopped.", true);
        }
    }

    protected final boolean isStopRequested() {
        return CalibrationSession.get().runStopRequested();
    }

    /** Buttons and editable values on the page while this run is active (see {@link Controls}). */
    protected final void setControls(Controls controls) {
        CalibrationSession.get().setControls(controls);
    }

    /** The next button pressed on the page, or null. */
    protected final String nextControl() {
        return CalibrationSession.get().nextControl();
    }

    /**
     * Whether the charts record. Off: the live numbers still update, but no chart points are added,
     * e.g. while the shooter wheel stands still. When recording starts again, the lines show a gap.
     */
    protected final void setRecording(boolean on) {
        CalibrationSession.get().setRecording(on);
    }

    /** Plots these live values together, one chart per group, so values of very different size get their own scale. */
    protected final void setCharts(String[]... groups) {
        java.util.List<Object> out = new java.util.ArrayList<>();
        for (String[] g : groups) out.add(new java.util.ArrayList<Object>(java.util.Arrays.asList((Object[]) g)));
        CalibrationSession.get().setCharts(out);
    }

    protected final boolean opModeIsActive() {
        return !isStopRequested();
    }

    protected final void sleep(long ms) throws InterruptedException {
        Thread.sleep(ms);
    }

    protected final void idle() {
        Thread.yield();
    }
}
