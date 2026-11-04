package org.firstinspires.ftc.teamcode.op;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.configuration.ShooterConfig;
import org.firstinspires.ftc.teamcode.hw.Shooter;

/**
 * Bench program for tuning the shooter. It does not touch the drive train, so the robot can stay on
 * the table. Follow the steps in docs/SHOOTER-PID.md.
 *
 * Open Panels at http://192.168.43.1:8001 while connected to the Control Hub's WiFi. Use the
 * Configurables tab for ShooterConfig and the Graph tab for the shooter_ values.
 *
 * Gamepad 1:
 *   A            spin at ShooterConfig.tuningTargetRpm with speed control
 *   B            off (the wheel coasts down)
 *   Y            fixed power tuningOpenLoopPower, for measuring kV
 *   dpad up/down target +/- dpadStepRpm
 *   X            drop the target by 500 RPM and back again, to watch the response without a ball
 *   left trigger run the intake to feed balls (if the robot has an intake_motor)
 */
@TeleOp(name = "Shooter Tuning", group = "Shooter")
public class ShooterTuning extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "outtake_motor");
        DcMotorEx intakeMotor = hardwareMap.tryGet(DcMotorEx.class, "intake_motor");
        Shooter shooter = new Shooter(hardwareMap, shooterMotor);

        telemetry.addLine("Shooter Tuning ready. A = spin, B = off, Y = fixed power");
        if (shooter.getFault() != null) {
            telemetry.addLine("FAULT: " + shooter.getFault());
        }
        telemetry.update();

        waitForStart();

        boolean loweredForStepTest = false;

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                loweredForStepTest = false;
                shooter.setTargetRpm(ShooterConfig.tuningTargetRpm);
            }
            if (gamepad1.bWasPressed()) {
                shooter.stop();
            }
            if (gamepad1.yWasPressed()) {
                shooter.setOpenLoopPower(ShooterConfig.tuningOpenLoopPower);
            }
            if (gamepad1.dpadUpWasPressed()) {
                ShooterConfig.tuningTargetRpm += ShooterConfig.dpadStepRpm;
                if (shooter.isOn()) shooter.setTargetRpm(ShooterConfig.tuningTargetRpm);
            }
            if (gamepad1.dpadDownWasPressed()) {
                ShooterConfig.tuningTargetRpm -= ShooterConfig.dpadStepRpm;
                if (shooter.isOn()) shooter.setTargetRpm(ShooterConfig.tuningTargetRpm);
            }
            if (gamepad1.xWasPressed() && shooter.isOn()) {
                loweredForStepTest = !loweredForStepTest;
                shooter.setTargetRpm(ShooterConfig.tuningTargetRpm - (loweredForStepTest ? 500 : 0));
            }

            if (intakeMotor != null) {
                intakeMotor.setPower(gamepad1.left_trigger > 0.5 ? ShooterConfig.tuningFeedPower : 0);
            }

            shooter.update();
            shooter.addTelemetry(telemetry);
            telemetry.addData("encoder_counts", shooter.readEncoderCounts()); // about 8192 per wheel turn
            telemetry.addData("velocity_window_ms", shooter.getAppliedWindowMs());

            // While running at fixed power: kV is the power per RPM needed to hold a speed.
            double measuredRpm = shooter.getMeasuredRpm();
            if (!shooter.isOn() && shooter.getPower() > 0 && measuredRpm > 500) {
                double voltsPerNominal = shooter.getBatteryVolts() / ShooterConfig.nominalVolts;
                double kVEstimate = (shooter.getPower() * voltsPerNominal - ShooterConfig.kS) / measuredRpm;
                telemetry.addData("kV_estimate_x1000", kVEstimate * 1000);
            }
            telemetry.update();

            // The full robot loop also runs odometry and the drive train. Imitate that here, so that
            // the gains we find on the bench also work on the full robot.
            sleep(ShooterConfig.tuningExtraLoopDelayMs);
        }
    }
}
