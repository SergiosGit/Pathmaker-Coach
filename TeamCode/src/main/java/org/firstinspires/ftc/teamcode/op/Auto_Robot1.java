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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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



    static FtcDashboard dashboard = FtcDashboard.getInstance();
    static Telemetry telemetry = dashboard.getTelemetry();
    DriveTrain driveTrain = new DriveTrain(this);

    @Override
    public void runOpMode() throws InterruptedException {
        //GameSetup.init("robot1","linear");
        //final TouchSensor limitSwitch;
        //limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");
        //final ColorRangeSensor colorRangeSensor;
        WebCam.init(this);
        while (WebCam.setManualExposure(6, 250, telemetry) == null) {
            Thread.sleep(100);
        }
        RobotPose.initializePose(this, driveTrain, telemetry);
        MyIMU.initMyIMU(this);
        MyIMU.updateTelemetry(telemetry);
        PathMakerStateMachine.setAutonomous();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        int cycles = 0;
        int TEST_CYCLES = 10;
        PathMakerStateMachine.initAuto();
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            telemetry.addData("thisPathNumber", thisPathNumber);
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
                    telemetry.addLine(String.format("ave/PM cycle %d /  %d (ms)", (int) t1, PathManager.PMcycleTime_ms));
                    telemetry.addLine(String.format("Path Goals forward/strafe/turn %.1f / %.1f / %.1f (in/deg)",
                            PathDetails.forwardGoal_in,
                            PathDetails.strafeGoal_in,
                            PathDetails.turnGoal_deg));
                    telemetry.addLine(String.format("RoboPose forward/strafe/turn %.1f / %.1f / %.1f (in/deg)",
                            RobotPose.getForward_in(),
                            RobotPose.getStrafe_in(),
                            RobotPose.getHeadingAngle_deg()));
                    telemetry.addLine(String.format("forward/strafe/turn velocities %.1f / %.1f / %.1f (in/deg)",
                            RobotPose.getForwardVelocity_inPerSec(),
                            RobotPose.getStrafeVelocity_inPerSec(),
                            RobotPose.getHeadingVelocity_degPerSec()));
                    telemetry.addLine(String.format("pathTime %.0f", PathDetails.pathTime_ms));
                    telemetry.addLine(String.format("elapsedTime %.0f", PathDetails.elapsedTime_ms.seconds()));
                    telemetry.addLine(String.format("inTargetZone %b", PathManager.inTargetZone));

                    telemetry.update();
                    cycles = 0;
                }
                PathMakerStateMachine.updateAuto();
            }
        }
    }
}
