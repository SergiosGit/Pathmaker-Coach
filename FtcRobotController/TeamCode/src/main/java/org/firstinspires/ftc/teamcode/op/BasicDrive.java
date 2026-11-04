package org.firstinspires.ftc.teamcode.op;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.configuration.RobotConfig;

@TeleOp
public class BasicDrive extends LinearOpMode {
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        RobotConfig.initializeRobotType(hardwareMap.appContext, true);
        frontLeft = hardwareMap.get(DcMotor.class, RobotConfig.DriveTrain.getFrontLeftMotorName());
        frontRight = hardwareMap.get(DcMotor.class, RobotConfig.DriveTrain.getFrontRightMotorName());
        backLeft = hardwareMap.get(DcMotor.class, RobotConfig.DriveTrain.getBackLeftMotorName());
        backRight = hardwareMap.get(DcMotor.class, RobotConfig.DriveTrain.getBackRightMotorName());

        frontLeft.setDirection(RobotConfig.DriveTrain.isFrontLeftReversed() ? DcMotorSimple.Direction.REVERSE: DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(RobotConfig.DriveTrain.isFrontRightReversed() ? DcMotorSimple.Direction.REVERSE: DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(RobotConfig.DriveTrain.isBackLeftReversed() ? DcMotorSimple.Direction.REVERSE: DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(RobotConfig.DriveTrain.isBackRightReversed() ? DcMotorSimple.Direction.REVERSE: DcMotorSimple.Direction.FORWARD);

        frontLeft.setPower(1);
        Thread.sleep(2000);
        frontLeft.setPower(0);
        frontRight.setPower(1);
        Thread.sleep(2000);
        frontRight.setPower(0);
        backLeft.setPower(1);
        Thread.sleep(2000);
        backLeft.setPower(0);
        backRight.setPower(1);
        Thread.sleep(2000);
        backRight.setPower(0);

        waitForStart();

        while (opModeIsActive()) {
            double forward = gamepad1.left_stick_y;
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
            telemetry.update();

            frontLeft.setPower(forward - strafe + turn);
            backLeft.setPower(forward + strafe + turn);
            frontRight.setPower(forward + strafe - turn);
            backRight.setPower(forward - strafe - turn);
//            }
        }
    }
}
