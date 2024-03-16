//
// PathDetails.java
//
// The PathDetails class implements the definition of robot paths. Each path is controlled by two
// constants for each of the three degrees of freedom (DOF). From a robot centric view the DOFs are
// forward, strafe, and turn. The two constants that define each DOF are:
// a goal (e.g. forwardGoal_in = 57) that defines the endpoint to be reached at the end of the path,
// and a delay (e.g. forwardDelay_ms = 500) that defines the timing from the beginning of the path
// at which the goal will be activated. The robot will attempt to reach the goal while maximizing
// the the use of the available battery power. The robot will ramp power down when reaching the goal.
// Additionally, the PathTime constant controls the total time allowed for the path. When PathTime
// is larger than the time needed to reach the goal, it has no effect. However, if PathTime is
// reached before the goal, the path is terminated at the current conditions. In other words, if
// the robot is moving it will continue to do so. This can be useful to create a "rolling stop"
// for a smooth transition to the next robot action.
// The coordinate system (COS) that PathMaker uses is the robot centric coordinate system where
// the forward direction is Y and the strafe direction is X. Rotations count positive going
// right. Whenever the encoders are reset, the COS will be reset to the current location.
//
// MIT License
// Copyright (c) 2023 bayrobotics.org
//
package org.firstinspires.ftc.teamcode.pathmaker;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hw.WebCam;
import org.firstinspires.ftc.teamcode.op.RobotPose;

import java.util.ArrayList;

@Config
public class PathDetails {
    // PathDetails defines each path goals and constraints for each DOF (x,y,a)
    // PathMaker uses a robot centric coordinate system (COS)
    // All x/y coordinates relate to the field center
    // Positive y is towards the backstage wall
    // Positive x is towards the right when looking towards the backstage wall
    // Positive turn is clockwise (tbc). Turn is also called "a" for angle
    public static double powerScaling = 1;
    public static double yFieldGoal_in, yFieldOffset_in =0;
    public static double xFieldGoal_in, xFieldOffset_in =0;
    public static double aFieldGoal_deg, aFieldOffset_deg =0;
    private static double xRelativeToTag, yRelativetoTag, aRelativetoTag;
    public static double xInitialPowerSignum = 1, yInitialPowerSignum = 1, aInitialPowerSignum = 1;
    // set initial delay
    public static double yFieldDelay_ms;
    public static double xFieldDelay_ms;
    public static double turnFieldDelay_ms;
    public static ElapsedTime elapsedTime_ms = new ElapsedTime();
    public static double pathTime_ms = 0;
    public  static PathMakerStateMachine.PM_STATE PMSMstate;

    public static double lastTurnGoal;

    public static void initializePath() {
        // initialize each new path
        pathTime_ms = 9E9;
        elapsedTime_ms.reset();
        powerScaling = 1;
        yFieldDelay_ms = 0;
        xFieldDelay_ms = 0;
        turnFieldDelay_ms = 0;
        PathMakerStateMachine.aprilTagDetectionOn = false;
        PMSMstate = PathMakerStateMachine.PM_STATE.AUTO_SET_PATH;
        PathManager.powerScalingTurn = 1;
        PathManager.maxPowerStepUp = 0.1;
        PathManager.inTargetZone = false;
        PathManager.yTargetZone_in = 1;
        PathManager.xTargetZone_in = 1;
        PathManager.turnTargetZone_deg = 1;
        PathManager.yRampReach_in = 24;
        PathManager.xRampReach_in = 24;
        PathManager.turnRampReach_deg = 45;
        PathManager.yMinVelocity_InchPerSec = 2;
        PathManager.xMinVelocity_InchPerSec = 2;
        PathManager.turnMinVelocity_degPerSec = 4;
        PathManager.rampType_x = PathManager.RAMPTYPE.LINEAR;
        PathManager.rampType_y = PathManager.RAMPTYPE.LINEAR;
        PathManager.rampType_a = PathManager.RAMPTYPE.LINEAR;
        PathManager.approachPowerTurn = 0.1;
        PathManager.approachPowerXY = 0.2;
        PathManager.breakPower = 0.05;
        PathManager.breakPowerScale = 0.5;
        //WebCam.stopWebcam();
    }
    public enum Path {
        // Each path is labeled with a name as defined in this enum
        P1, P11, P2, P3, P4,
        AUTO_BACKBOARD, AUTO_APRIL_TAG, AUTO_PIXEL_STACKS,
        DRIVER_CONTROLLED,
        DONE
    }   // end enum Event
    public static ArrayList<Path> autoPathList;
    public static void initAutoPathList() {
        PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_SET_PATH;
        PathMakerStateMachine.currentPath = PathMakerStateMachine.nextPath = 0;
        PathMakerStateMachine.aprilTagDetectionOn = false;
        PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
        autoPathList = new ArrayList<PathDetails.Path>();
        autoPathList.add(PathDetails.Path.P1);
        autoPathList.add(PathDetails.Path.P11);
        autoPathList.add(PathDetails.Path.AUTO_BACKBOARD);
        autoPathList.add(PathDetails.Path.AUTO_BACKBOARD);
//        autoPathList.add(PathDetails.Path.P2);
//        autoPathList.add(PathDetails.Path.P3);
//        autoPathList.add(PathDetails.Path.P4);
//        autoPathList.add(PathDetails.Path.AUTO_PIXEL_STACKS);
    }   // end method initAuto

    public static void setPath(Path path, Telemetry telemetry) throws InterruptedException {
        // set path parameters
        // this method is called from PathMakerStateMachine
        // In Autonomous mode, the autoPathList is set by the PathMakerStateMachine
        // In Tele mode, the path is set to DRIVER_CONTROLLED in the PathMakerStateMachine
        initializePath();
        switch (path) {
            case DRIVER_CONTROLLED:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.TELEOP;
                PathManager.maxPowerStepUp = 0.1;
                PathManager.turnTargetZone_deg = 3;
                powerScaling = 1;
                break;
            case P1:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                powerScaling = 0.8;
                xFieldGoal_in = -12; yFieldGoal_in = 0; aFieldGoal_deg = 0;
                yFieldDelay_ms = 700;
                //PathManager.rampType_y = PathManager.RAMPTYPE.STEP;
                PathManager.yTargetZone_in = 2;
                PathManager.xTargetZone_in = 1;
                PathManager.turnTargetZone_deg = 2;
                PathManager.yMinVelocity_InchPerSec = 7;
                PathManager.xMinVelocity_InchPerSec = 2;
                PathManager.approachPowerTurn = 0.1;
                PathManager.approachPowerXY = 0.25;
                PathManager.yRampReach_in = 0;
                calculateInitialPowerSignum(xFieldGoal_in, yFieldGoal_in, aFieldGoal_deg);
                break;
            case P11:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                powerScaling = 0.8;
                xFieldGoal_in = -12; yFieldGoal_in = 48; aFieldGoal_deg = 0;
                PathManager.rampType_y = PathManager.RAMPTYPE.STEP;
                PathManager.yTargetZone_in = 2;
                PathManager.xTargetZone_in = 1;
                PathManager.turnTargetZone_deg = 2;
                PathManager.yMinVelocity_InchPerSec = 7;
                PathManager.xMinVelocity_InchPerSec = 2;
                PathManager.approachPowerTurn = 0.1;
                PathManager.approachPowerXY = 0.25;
                PathManager.yRampReach_in = 24;
                PathManager.xRampReach_in = 24;
                calculateInitialPowerSignum(xFieldGoal_in, yFieldGoal_in, aFieldGoal_deg);
                break;
            case P2:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                PathManager.yRampReach_in = 0;
                powerScaling = 0.5;
                xFieldGoal_in = -12; yFieldGoal_in = 12; aFieldGoal_deg = 0;
                calculateInitialPowerSignum(xFieldGoal_in, yFieldGoal_in, aFieldGoal_deg);
                break;
            case P3:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                PathManager.yRampReach_in = 0;
                powerScaling = 0.5;
                xFieldGoal_in = -12; yFieldGoal_in = -6; aFieldGoal_deg = 0;
                calculateInitialPowerSignum(xFieldGoal_in, yFieldGoal_in, aFieldGoal_deg);
                break;
            case P4:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                PathManager.yRampReach_in = 5;
                powerScaling = 0.5;
                xFieldGoal_in = -12; yFieldGoal_in = -10; aFieldGoal_deg = 0;
                break;
            case AUTO_BACKBOARD:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_APRILTAG;
                WebCam.streamWebcam(WebCam.WEBCAM.WEBCAM1, telemetry);
                PathMakerStateMachine.aprilTagDetectionOn = true;
                PathMakerStateMachine.aprilTagDetectionID = 2;
                powerScaling = 0.4;
                PathManager.rampType_y = PathManager.RAMPTYPE.STEP;
                PathManager.yTargetZone_in = 2;
                PathManager.xTargetZone_in = 1;
                PathManager.yRampReach_in = 12;
                PathManager.turnTargetZone_deg = 2;
                PathManager.yMinVelocity_InchPerSec = 8;
                PathManager.xMinVelocity_InchPerSec = 2;
                PathManager.approachPowerTurn = 0.1;
                PathManager.approachPowerXY = 0.25;
                PathManager.breakPowerScale = 0.9;
                // in auto mode the goals are relative to the tag positions (see autoAprilTag)
                yRelativetoTag = -10;
                xRelativeToTag = 0;
                aRelativetoTag = 0;
                break;
            case AUTO_PIXEL_STACKS:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_APRILTAG;
                WebCam.streamWebcam(WebCam.WEBCAM.WEBCAM2, telemetry);
                PathMakerStateMachine.aprilTagDetectionOn = true;
                PathMakerStateMachine.aprilTagDetectionID = 9;
                powerScaling = 0.5;
                yRelativetoTag = 14;
                xRelativeToTag = 0;
                aRelativetoTag = 0;
                break;
            case DONE:
                break;
        }
    }

    private static void calculateInitialPowerSignum(double goalX, double goalY, double goalA) {
        // calculate initial power signum (motor direction). The PathManager uses this when
        // rampReach is set to zero. In this case the robot will keep moving until with
        // full power until the goal is reached.
        xInitialPowerSignum = Math.signum( goalX - RobotPose.getFieldX_in());
        yInitialPowerSignum = Math.signum( goalY - RobotPose.getFieldY_in());
        aInitialPowerSignum = Math.signum( goalA - RobotPose.getFieldAngle_deg());
    }

    // check for AprilTag detection
    public static void autoAprilTagAndFieldGoals() {
        // AprilTag detection while the robot is stopped (investigate if robot can move slowly)
        // This method will rebase the coordinate system to the detected tag
        // and set the field goals relative to the tag position
        // The robot will then move to these field goals
        // This method is called from PathMakerStateMachine
        int xyWebCamMultiplier = 1;
        if (WebCam.currentWebCam == WebCam.WEBCAM.WEBCAM2) {
            xyWebCamMultiplier = -1;
        }
        PathManager.maxPowerStepUp = 0.1;
        //pathTime_ms = PathManager.timeStep_ms;
        powerScaling = 0.6;
        // need to update with actual robot position
        double distanceToTarget = WebCam.distanceToTarget;
        double a = WebCam.angleToTarget; // yaw angle
        double x = -distanceToTarget * Math.sin(Math.toRadians(a)) * xyWebCamMultiplier;
        // limit sideways motion to 5 inches
        if (Math.abs(x) > 5) {
            x = 5 * Math.signum(x);
        } else  {
            x = WebCam.offsetToTarget;
        }
        double y = distanceToTarget * Math.cos(Math.toRadians(a)) * xyWebCamMultiplier;
        double tagXYA[] = RobotPose.rebaseRelativeToTag(x, y, a, PathMakerStateMachine.aprilTagDetectionID);
        // position relative to tags
        yFieldGoal_in = tagXYA[0] + yRelativetoTag;
        xFieldGoal_in = tagXYA[1] + xRelativeToTag;
        aFieldGoal_deg = tagXYA[2] + aRelativetoTag;
        yFieldDelay_ms = 0;
        xFieldDelay_ms = 0;
        turnFieldDelay_ms = 0;
        lastTurnGoal = aFieldGoal_deg;
        PathManager.yRampReach_in = 24;
        PathManager.xRampReach_in = 12;
        PathManager.turnRampReach_deg = 45;
        //WebCam.stopWebcam();
    }
}