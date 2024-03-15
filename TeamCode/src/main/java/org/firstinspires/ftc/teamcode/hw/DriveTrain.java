// DriveTrain.java
//
// MIT License
// Copyright (c) 2023 bayrobotics.org
//
package org.firstinspires.ftc.teamcode.hw;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.op.RobotPose;
import org.firstinspires.ftc.teamcode.pathmaker.GameSetup;

import java.util.List;

public class DriveTrain {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private static DcMotorEx frontLeft   = null;
    private static DcMotorEx backLeft  = null;
    private static DcMotorEx backRight = null;
    private static DcMotorEx frontRight = null;
    private static GameSetup.RobotModel robotModel = GameSetup.RobotModel.ROBOT1;
    public static List <LynxModule> allHubs;
    public DriveTrain(LinearOpMode opMode){
        myOpMode = opMode;
    }
    public void init() throws InterruptedException {
        // For bulk read make sure to use DcMotorEx when instantiating motors.
        frontLeft = myOpMode.hardwareMap.get(DcMotorEx.class,"front_left");
        backLeft = myOpMode.hardwareMap.get(DcMotorEx.class,"back_left");
        backRight = myOpMode.hardwareMap.get(DcMotorEx.class,"back_right");
        frontRight = myOpMode.hardwareMap.get(DcMotorEx.class,"front_right");
        initMotor(frontLeft);
        initMotor(backLeft);
        initMotor(backRight);
        initMotor(frontRight);
        // if all motors are plugged in the same you may need to do:
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        // Get access to a list of Expansion Hub Modules to enable changing caching methods.
        allHubs = myOpMode.hardwareMap.getAll(LynxModule.class);
        // Set all Expansion hubs to use the MANUAL Bulk Caching mode
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

    }

    public DcMotorEx initMotor(DcMotorEx motor) throws InterruptedException {
        //motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Thread.sleep(100);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    public static void setMotorPowers(double fl, double bl, double br, double fr){
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        backRight.setPower(br);
        frontRight.setPower(fr);
    }

    public static double[] getMotorCurrents() {
        return new double[]{
                // note, this is not bulk-read
                frontLeft.getCurrent(CurrentUnit.AMPS),
                backLeft.getCurrent(CurrentUnit.AMPS),
                backRight.getCurrent(CurrentUnit.AMPS),
                frontRight.getCurrent(CurrentUnit.AMPS)
        };
    }

    public static double[] getMotorVelocities() {
        return new double[]{
                frontLeft.getVelocity(AngleUnit.DEGREES),
                backLeft.getVelocity(AngleUnit.DEGREES),
                backRight.getVelocity(AngleUnit.DEGREES),
                frontRight.getVelocity(AngleUnit.DEGREES)
        };
    }
    public static int[] getEncoderValues() {
        if (RobotPose.odometry == RobotPose.ODOMETRY.XYPLUSIMU) {
            // odometry from wheel encoders: 0: frontLeft, 1: backLeft, 2: backRight, 3: frontRight
            return new int[]{frontLeft.getCurrentPosition(), backLeft.getCurrentPosition(), backRight.getCurrentPosition(), frontRight.getCurrentPosition()};
        } else if (RobotPose.odometry == RobotPose.ODOMETRY.DEADWHEEL) {
            // dead wheel odometry: 0: left encoder, 1: middle encoder, 2: right encoder
            return new int[]{-frontLeft.getCurrentPosition(), backRight.getCurrentPosition(), -frontRight.getCurrentPosition()};
        } else {
            return new int[]{0, 0, 0, 0};
        }
    }
}
