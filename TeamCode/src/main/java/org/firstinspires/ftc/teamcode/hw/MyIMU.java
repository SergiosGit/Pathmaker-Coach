// MIT License
// Copyright (c) 2023 bayrobotics.org
//
package org.firstinspires.ftc.teamcode.hw;
//import com.qualcomm.hardware.bosch.BHI260IMU;
//import com.qualcomm.hardware.bosch.BNO055IMU;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class MyIMU {
    // see https://stemrobotics.cs.pdx.edu/node/7266.html
    //BNO055IMU imu;
    //BHI260IMU imu2;
    static  IMU imu;
    Orientation             lastAngles = new Orientation();
    static double globalAngle;
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    public MyIMU(LinearOpMode opMode){
        myOpMode = opMode;
    }
    public void setOpMode(LinearOpMode opMode){
        myOpMode = opMode;
    }
    public static YawPitchRollAngles orientation;
    public static AngularVelocity angularVelocity;
    private static double thisAngle_rad = 0;
    public static double thisAngle_deg = 0;
    private static double lastAngle_deg = 0;
    private static double thisIMU_deg = 0, lastIMU_deg = 0;
    private static int n360 = 0;


    public static void initMyIMU(LinearOpMode opMode){
        // Retrieve the IMU from the hardware map
        imu = opMode.hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        lastAngle_deg = thisAngle_deg = 0;
        lastIMU_deg = thisIMU_deg = 0;
        n360 = 0;
        resetAngle();
    }
    public static void resetAngle()
    {
        globalAngle = 0;
        imu.resetYaw();
    }
    public static void updateTelemetry(Telemetry telemetry){
        // Retrieve Rotational Angles and Velocities
        // Retrieve Rotational Angles and Velocities
        orientation = imu.getRobotYawPitchRollAngles();
        angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        telemetry.addLine(String.format("IMU Yaw/Pitch/Roll: %.2f %.2f %.2f deg",
                orientation.getYaw(AngleUnit.DEGREES),
                orientation.getPitch(AngleUnit.DEGREES),
                orientation.getRoll(AngleUnit.DEGREES)));
        telemetry.addLine(String.format("IMU Yaw/Pitch/Roll velocities: %.2f %.2f %.2f deg/sec",
                angularVelocity.zRotationRate,
                angularVelocity.xRotationRate,
                angularVelocity.yRotationRate));
        telemetry.addData("thisAngle_deg", thisAngle_deg);
    }
    public double getAngle_deg()
    {
        // X (firstAngle) axis is for heading with controller logo to the left and USB backwards
        // Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        thisIMU_deg = -angles.getYaw(AngleUnit.DEGREES);
        if (lastIMU_deg > 140 && thisIMU_deg < -140) {
            n360++;
        } else if (lastIMU_deg < -140 && thisIMU_deg > 140) {
            n360--;
        }
        thisAngle_deg = thisIMU_deg + n360 * 360;
        lastIMU_deg = thisIMU_deg;
        return thisAngle_deg;
    }
    public double getAngle_rad(){
        thisAngle_rad = getAngle_deg() / 180 * Math.PI;
        return thisAngle_rad;
    }
}
