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

    final int       TEST_CYCLES    = 10;

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
        MyIMU.initMyIMU(this);
        PathMakerStateMachine.setDriverControlled();
        PathDetails.initializePath();
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive()) {
            if (isStopRequested()) return;
            PathMakerStateMachine.updateTele(gamepad1);
            cycles++;
            if (cycles > TEST_CYCLES) {
                double t1 = timer.milliseconds() / cycles;
                timer.reset();
                telemetry.addLine(String.format("inTargetZone %b", PathManager.inTargetZone));
                telemetry.addLine(String.format("ave/PM cycle %d /  %d (ms)", (int) t1, PathManager.PMcycleTime_ms));
                telemetry.addLine(String.format("forward/strafe/turn goal %.1f / %.1f / %.1f (in/deg)",
                        PathDetails.forwardGoal_in,
                        PathDetails.strafeGoal_in,
                        PathDetails.turnGoal_deg));
                telemetry.addLine(String.format("RoboPose forward/strafe/turn %.1f / %.1f / %.1f (in/deg)",
                        RobotPose.getForward_in(),
                        RobotPose.getStrafe_in(),
                        RobotPose.getHeadingAngle_deg()));
                telemetry.addLine(String.format("fl/bl/br/fr motor current %.1f / %.1f / %.1f / %.1f (A)",
                        RobotPose.motorCurrents[0],
                        RobotPose.motorCurrents[1],
                        RobotPose.motorCurrents[2],
                        RobotPose.motorCurrents[3]));
                telemetry.addLine(String.format("fl/bl/br/fr motor velocity %.1f / %.1f / %.1f / %.1f (ticks/s)",
                        RobotPose.motorVelocities[0],
                        RobotPose.motorVelocities[1],
                        RobotPose.motorVelocities[2],
                        RobotPose.motorVelocities[3]));
                MyIMU.updateTelemetry(telemetry);
                WebCam.telemetryAprilTag(telemetry);
                telemetry.addData("state", PathMakerStateMachine.state);
                telemetry.update();
                cycles = 0;
            }
        }
    }
}
