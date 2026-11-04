package org.firstinspires.ftc.teamcode.hw;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Drivetrain {

    private Follower follower;
    private Gamepad gamepad;
    private TelemetryManager telemetry;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    public Drivetrain(Robot robot, Gamepad gamepad) {
        this.gamepad = gamepad;
        this.follower = robot.follower;
        this.telemetry = robot.telemetry;

        frontLeft = robot.hw.get(DcMotor.class, "front_left");
        frontRight = robot.hw.get(DcMotor.class, "front_right");
        backLeft = robot.hw.get(DcMotor.class, "back_left");
        backRight = robot.hw.get(DcMotor.class, "back_right");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // follower = Constants.create(robot.hw);

        telemetry = robot.telemetry;
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public Command update() {
        return infinite(() -> {

            DrivePowers powers = ManualDrive.fieldCentric(
                    -signedSquare(gamepad.left_stick_y),
                    signedSquare(gamepad.left_stick_x),
                    signedSquare(gamepad.right_stick_x),
                    follower.pose().heading()
            );

            // if(Math.abs(gamepad.right_stick_x) < 0.1) powers = ManualDrive.headingLock(follower, Constants.foresightConfig.headingFeedback.get(), powers, targetHeading);
            // else targetHeading = follower.pose().heading();
            ManualDrive.driveOrHold(follower, powers);

            telemetry.addLine(String.format("Robot Pose x/y/a %6.1f %6.1f %6.1f",
                    follower.pose().x(),
                    follower.pose().y(),
                    Math.toDegrees(follower.pose().heading())));
        });
    }

    public double signedSquare(double num) {
        return Math.signum(num) * Math.pow(num, 2);
    }
}
