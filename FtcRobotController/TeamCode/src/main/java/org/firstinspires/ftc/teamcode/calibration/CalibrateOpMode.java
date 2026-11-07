package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.calibration.runner.CalibrationSession;

/**
 * Runs calibration steps chosen on the web page (http://192.168.43.1:8090). Stop on the Driver
 * Hub ends any step at once.
 */
@TeleOp(name = "Calibrate", group = "calibration")
public class CalibrateOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        CalibrationSteps.install();
        CalibrationSession session = CalibrationSession.get();

        telemetry.addLine("Press ▶, then use the calibration page:");
        telemetry.addLine("http://192.168.43.1:" + CalibrationServer.PORT);
        telemetry.update();
        waitForStart();

        session.attach(new CalibrationSession.Host() {
            @Override public boolean stopRequested() { return isStopRequested(); }
            @Override public HardwareMap hardwareMap() { return hardwareMap; }
            @Override public Gamepad gamepad1() { return gamepad1; }
            @Override public Gamepad gamepad2() { return gamepad2; }
        });
        try {
            while (opModeIsActive()) {
                session.runPending();
                telemetry.addLine("Calibration page: http://192.168.43.1:" + CalibrationServer.PORT);
                telemetry.addData("Step", session.state().get("title"));
                telemetry.addData("Status", session.state().get("phase"));
                telemetry.update();
                sleep(50);
            }
        } finally {
            session.detach();
        }
    }
}
