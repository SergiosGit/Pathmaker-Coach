package org.firstinspires.ftc.teamcode.calibration.runner;

import org.firstinspires.ftc.teamcode.calibration.Calibration;

/**
 * One calibration step. Has the same calls as Pedro AutoTune's Procedure, so Pedro's tuning
 * procedures run here unchanged; prompts and results go to our web page instead.
 *
 * Results whose "prefix.name" is a calibration key are saved on the Control Hub immediately.
 */
public abstract class Procedure {

    public enum Language { JAVA, KOTLIN, JSON }

    public final String name;
    public final String description;
    private String resultPrefix = "";

    public Procedure(String name, String description) {
        this.name = name;
        this.description = description;
    }

    public abstract void run() throws InterruptedException;

    final void setResultPrefix(String prefix) { this.resultPrefix = prefix; }

    protected void confirmation(String title, String message) throws InterruptedException {
        CalibrationSession.get().awaitConfirmation(title, message);
    }

    protected Inputs inputs(String title, String message) {
        return new Inputs(title, message);
    }

    protected void awaitInputs(Inputs inputs) throws InterruptedException {
        CalibrationSession.get().awaitInputs(inputs);
    }

    protected final void abort(String message) throws InterruptedException {
        throw new CalibrationAbort(message, false);
    }

    protected void withDisplay(Display display, InterruptibleBlock block) throws InterruptedException {
        CalibrationSession s = CalibrationSession.get();
        s.setDisplay(display);
        try {
            block.run();
        } finally {
            s.setDisplay(null);
        }
    }

    /** Shows a result; saves it if "prefix.name" (or the name itself) is a calibration key. */
    protected final void result(String name, Object value) {
        String key = Calibration.isDefined(name) || resultPrefix.isEmpty() ? name : resultPrefix + "." + name;
        CalibrationSession.get().result(key, name, value);
    }

    /** Pedro's procedures print code to paste. Not needed here: results are saved directly. */
    protected final void code(Language language, String code) { }

    protected final void code(String language, String code) { }

    protected final <R> R runOpMode(TuningOpMode<R> opMode) throws InterruptedException {
        return CalibrationSession.get().runOpMode(opMode);
    }

    public interface InterruptibleBlock {
        void run() throws InterruptedException;
    }
}
