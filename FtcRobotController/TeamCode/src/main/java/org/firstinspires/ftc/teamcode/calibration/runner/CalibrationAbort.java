package org.firstinspires.ftc.teamcode.calibration.runner;

/** Ends the running calibration step: Abort on the page, Stop on the Driver Hub, or a failed check. */
public class CalibrationAbort extends RuntimeException {
    public final boolean byUser;

    public CalibrationAbort(String message, boolean byUser) {
        super(message);
        this.byUser = byUser;
    }
}
