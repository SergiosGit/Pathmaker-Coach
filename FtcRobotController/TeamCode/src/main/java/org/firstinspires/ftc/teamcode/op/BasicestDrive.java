package org.firstinspires.ftc.teamcode.op;

import android.provider.Settings;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configuration.RobotConfig;

@TeleOp
public class BasicestDrive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        RobotConfig.initializeRobotType(hardwareMap.appContext, false);
        waitForStart();

        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            if (gamepad1.a) {
                forward *= 0.5;
                strafe *= 0.5;
                turn *= 0.5;
            }

            telemetry.addData("forward", forward);
            telemetry.addData("strafe", strafe);
            telemetry.addData("turn", turn);
            String androidId = Settings.Secure.getString(hardwareMap.appContext.getContentResolver(), Settings.Secure.ANDROID_ID);
            telemetry.addData("Android ID: ", androidId);
            telemetry.update();

//            }
        }
    }
}
