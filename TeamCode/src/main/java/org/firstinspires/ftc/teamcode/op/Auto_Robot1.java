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

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hw.MyIMU;
import org.firstinspires.ftc.teamcode.hw.WebCam;
import org.firstinspires.ftc.teamcode.pathmaker.PathDetails;
import org.firstinspires.ftc.teamcode.pathmaker.PathMakerStateMachine;
import org.firstinspires.ftc.teamcode.pathmaker.PathManager;
import org.firstinspires.ftc.teamcode.hw.DriveTrain;

@Config
@Autonomous
public class Auto_Robot1 extends LinearOpMode {

    public static double thisForwardPower = 0;
    public static double thisStrafePower = 0;
    public static double thisTurnPower = 0.2;
    public static double thisHeadingDrive = 0;
    public static int thisPathNumber = 1;
    public static int runTest_ms = 100;
    public static int thisNumberSteps = 2;
    public static int setZone = 1;



    //static FtcDashboard dashboard = FtcDashboard.getInstance();
    //static Telemetry telemetry = dashboard.getTelemetry();
    DriveTrain driveTrain = new DriveTrain(this);

    @Override
    public void runOpMode() throws InterruptedException {
        //GameSetup.init("robot1","linear");
        //final TouchSensor limitSwitch;
        //limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");
        //final ColorRangeSensor colorRangeSensor;
        WebCam.init(this, telemetry);
        RobotPose.initializePose(this, driveTrain, telemetry);
        MyIMU.initMyIMU(this);
        MyIMU.updateTelemetry(telemetry);
        PathMakerStateMachine.setAutonomous();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        int cycles = 0;
        int TEST_CYCLES = 10;
        PathMakerStateMachine.initAuto();
        telemetry.addData("thisPathNumber", thisPathNumber);
        telemetry.update();
        RobotPose.rebase(-12, 0, 0, 2); // start in front of tag 2, Y/X are relative to tag 2
        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) return;
            if (thisPathNumber == -1) {
                WheelPowerManager.setDrivePower(driveTrain, thisForwardPower, thisStrafePower, thisTurnPower, thisHeadingDrive);
                sleep(runTest_ms);
                WheelPowerManager.setDrivePower(driveTrain, 0, 0, 0, 0);
                RobotPose.readPose();
            } else if (thisPathNumber == 0) {
                // use dashboard parameters
                PathManager.moveRobot();
                sleep(runTest_ms);
            } else if (thisPathNumber == 1) {
                cycles++;
                if (cycles > TEST_CYCLES) {
                    double t1 = timer.milliseconds() / cycles;
                    timer.reset();
                    telemetry.addData("State", PathMakerStateMachine.state);
                    telemetry.addData("aprilTagDetectionOn", PathMakerStateMachine.aprilTagDetectionOn);
                    telemetry.addData("PathDetails.currentPath", PathMakerStateMachine.currentPath < 0? -1: PathMakerStateMachine.autoPathList.get(PathMakerStateMachine.currentPath));
                    //telemetry.addData(("number of detected tags"), WebCam.currentDetections==null?0:WebCam.currentDetections.size());
                    telemetry.addLine(String.format("inTargetZone %b", PathManager.inTargetZone));
                    telemetry.addLine(String.format("ave/PM cycle %d /  %d (ms)", (int) t1, PathManager.PMcycleTime_ms));
                    telemetry.addLine(String.format("Path Goals f/s/t %.1f / %.1f / %.1f (in/deg)",
                            PathDetails.forwardGoal_in,
                            PathDetails.strafeGoal_in,
                            PathDetails.turnGoal_deg));
                    telemetry.addLine(String.format("RoboPose f/s/t %.1f / %.1f / %.1f (in/deg)",
                            RobotPose.getForward_in(),
                            RobotPose.getStrafe_in(),
                            RobotPose.getHeadingAngle_deg()));
                    MyIMU.updateTelemetry(telemetry);
                    WebCam.telemetryAprilTag(telemetry);
                    telemetry.update();
                    cycles = 0;
                }
                PathMakerStateMachine.updateAuto();
            }
        }
    }
}
