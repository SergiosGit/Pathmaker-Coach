package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class BasicDrive extends LinearOpMode {
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            double forward = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
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
//            if (gamepad1.y) {
//                turn = 360;
//                frontLeft.setPower();
//            }
        }
    }
}
