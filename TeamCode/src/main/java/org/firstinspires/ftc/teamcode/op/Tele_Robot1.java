//
// PathMaker.java
//
// This is the main module showing the use of the PathMaker library that can be used
// for autonomous control of a Mecanum wheel drive train for the FTC competition.
// This implementation uses the FTC Dashboard for developing the robot path settings
// including simulating robot movements that can be displayed on the dashboard.
//
// MIT License
// Copyright (c) 2023 bayrobotics.org
//
package org.firstinspires.ftc.teamcode.op;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hw.DriveTrain;
import org.firstinspires.ftc.teamcode.hw.MyIMU;
import org.firstinspires.ftc.teamcode.hw.WebCam;
import org.firstinspires.ftc.teamcode.pathmaker.PathDetails;
import org.firstinspires.ftc.teamcode.pathmaker.PathMakerStateMachine;
import org.firstinspires.ftc.teamcode.pathmaker.PathManager;
@Config
@TeleOp (name = "Field Centric Driving", group = "Tests")
public class Tele_Robot1 extends LinearOpMode {

    public static double thisForwardPower = 0;
    public static double thisStrafePower = 0;
    public static double thisTurnPower = 0.2;
    public static double thisHeadingDrive = 0;
    public static int thisPathNumber = 0;
    public static int runTest_ms = 100;
    public static boolean runDriveTest = true;
    public static int thisNumberSteps = 2;
    public static int setZone = 1;
    public static double motorPowerMultiplier = 1.0;
    public static double turnDamping = 0.5;
    public static double gampadChange;
    public static double yPower, yPowerLast;
    public static double xPower, xPowerLast;
    public static double turnPower, turnPowerLast;

    final int TEST_CYCLES    = 10;

    static FtcDashboard dashboard = FtcDashboard.getInstance();
    static Telemetry telemetry = dashboard.getTelemetry();
    DriveTrain driveTrain = new DriveTrain(this);

    @Override
    public void runOpMode() throws InterruptedException {
        //GameSetup.init("robot1","linear");
        //final TouchSensor limitSwitch;
        //limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");
        //final ColorRangeSensor colorRangeSensor;
        int cycles = 0;
        WebCam.init(this, telemetry);
        RobotPose.initializePose(this, driveTrain, telemetry);
        RobotPose.setPose(0, 0, 0);
        MyIMU.init(this);
        PathMakerStateMachine.setDriverControlled();
        PathDetails.initializePath();
        MyIMU.updateTelemetry(telemetry);
        telemetry.addData("webcam", WebCam.webcamMessage);
        WebCam.telemetryAprilTag(telemetry);
        telemetry.update();
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive()) {
            if (isStopRequested()) return;
            cycles++;
            if (cycles > TEST_CYCLES) {
                double t1 = timer.milliseconds() / cycles;
                timer.reset();
                telemetry.addData("State", PathMakerStateMachine.pm_state);
                telemetry.addData("PathDetails.currentPath", PathMakerStateMachine.currentPath < 0? -1: PathDetails.autoPathList.get(PathMakerStateMachine.currentPath));
                telemetry.addLine(String.format("tag detection %b, ID %d, in zone %b", PathMakerStateMachine.aprilTagDetectionOn, PathMakerStateMachine.aprilTagDetectionID, PathManager.inTargetZone));
                telemetry.addLine(String.format("PathDetails.elapsedTime_ms %.1f", PathDetails.elapsedTime_ms.milliseconds()));
                telemetry.addLine(String.format("ave tele/PM cycle %d /  %d (ms)", (int) t1, PathManager.PMcycleTime_ms));
                double [] xya = RobotPose.tagOffset(PathMakerStateMachine.aprilTagDetectionID);
                telemetry.addLine(String.format("tagOffset x/y/a %.1f / %.1f / %.1f (in/deg)",
                        xya[0],
                        xya[1],
                        xya[2]));
                telemetry.addLine(String.format("Path Goals x/y/a %.1f / %.1f / %.1f (in/deg)",
                        PathDetails.yFieldGoal_in,
                        PathDetails.xFieldGoal_in,
                        PathDetails.aFieldGoal_deg));
                telemetry.addLine(String.format("RoboPose x/y/a %.1f / %.1f / %.1f (in/deg)",
                        RobotPose.getFieldY_in(),
                        RobotPose.getFieldX_in(),
                        RobotPose.getFieldAngle_deg()));
                // gamepad x/y input
                telemetry.addLine(String.format("gamepad Y/X/A %.1f / %.1f / %.1f",
                        gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x));
                // telemetry velocity
                telemetry.addLine(String.format("x/y/a velocity %.1f / %.1f / %.1f ",
                        RobotPose.getForwardVelocity_inPerSec(),
                        RobotPose.getStrafeVelocity_inPerSec(),
                        RobotPose.getHeadingVelocity_degPerSec()));
                telemetry.addLine(String.format("actual power Y/X/A %.1f / %.1f / %.1f",
                        yPower,
                        xPower,
                        turnPower));
                telemetry.addLine(String.format("last power Y/X/A %.1f / %.1f / %.1f",
                        yPowerLast,
                        xPowerLast,
                        turnPowerLast));
                MyIMU.updateTelemetry(telemetry);
                telemetry.addData("webcam", WebCam.webcamMessage);
                WebCam.telemetryAprilTag(telemetry);
                telemetry.update();
                cycles = 0;
            }
            PathMakerStateMachine.updateTele(gamepad1,telemetry);
            xPower = PathManager.xPower;
            yPower = PathManager.yPower;
            turnPower = PathManager.turnPower;
            xPowerLast = PathManager.xPowerLast;
            yPowerLast = PathManager.yPowerLast;
            turnPowerLast = PathManager.turnPowerLast;
        }
    }
}
