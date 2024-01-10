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

import static org.firstinspires.ftc.teamcode.pathmaker.PathMakerStateMachine.gamepadForward;
import static org.firstinspires.ftc.teamcode.pathmaker.PathMakerStateMachine.gamepadTurn;
import static org.firstinspires.ftc.teamcode.pathmaker.PathMakerStateMachine.gamepadStrafe;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hw.WebCam;
import org.firstinspires.ftc.teamcode.op.RobotPose;

@Config
public class PathDetails {
    // Note: the offsets defined below are used by the PathManager and are useful to set an offset
    // from an arbitrary origin when using the FTC Dashboard. The default (offset=0) defines the
    // robot centric coordinate system at the beginning of the path.
    public static double powerScaling = 1;
    public static double yFieldGoal_in, forwardOffset_in=0;
    public static double xFieldGoal_in, strafeOffset_in=0;
    public static double aFieldGoal_deg, turnOffset_deg=0;
    private static double xRelativeToTag, yRelativetoTag, aRelativetoTag;
    // set initial delay
    public static double yFieldDelay_ms;
    public static double xFieldDelay_ms;
    public static double turnFieldDelay_ms;
    public static double turnSensitivity = 0.4;
    public static ElapsedTime elapsedTime_ms = new ElapsedTime();
    public static double pathTime_ms = 0;
    public  static PathMakerStateMachine.State PMSMstate;

    public static double lastTurnGoal; // RobotPose.getHeadingAngle_deg()

    public static void initializePath() {
        // initialize
        pathTime_ms = 9E9;
        elapsedTime_ms.reset();
        powerScaling = 1;
        yFieldGoal_in = 0;
        xFieldGoal_in = 0;
        aFieldGoal_deg = 0;
        yFieldDelay_ms = 0;
        xFieldDelay_ms = 0;
        turnFieldDelay_ms = 0;
        PathMakerStateMachine.aprilTagDetectionOn = false;
        PMSMstate = PathMakerStateMachine.State.AUTO_SET_PATH;
        PathManager.maxPowerStepUp = 0.1;
        PathManager.inTargetZone = false;
        PathManager.forwardTargetZone_in = 1;
        PathManager.strafeTargetZone_in = 1;
        PathManager.turnTargetZone_deg = 1;
        PathManager.forwardRampReach_in = 11;
        PathManager.strafeRampReach_in = 11;
        PathManager.turnRampReach_deg = 45;
        PathManager.forwardMinVelocity_inPerSec = 2;
        PathManager.strafeMinVelocity_inPerSec = 2;
        PathManager.turnMinVelocity_degPerSec = 4;
        WebCam.stopWebcam();
    }
    public enum Path {
        P1, P2, P3, BACKBOARD,
        DriverControlled,
        AutoAprilTag, PIXELSTACKS, DONE
    }   // end enum Event

    public static void setPath(Path path) throws InterruptedException {
        initializePath();
        switch (path) {
            case DriverControlled:
                PathManager.maxPowerStepUp = 0.1;
                //pathTime_ms = PathManager.timeStep_ms;
                powerScaling = 1;
                // need to update with actual robot position
                //robotAngle = RobotPose.getHeadingAngle_deg();
                // Note that "forward" is directly from the gamepad, i.e. a value between -1 and 1.
                // Robot forward power is maximum for requested gaols larger or equal than forwardRampReach_in (this is controlled by the PathManager)
                // Similar for strafe and turn.
                yFieldGoal_in = PathManager.forwardRampReach_in * gamepadForward + RobotPose.getFieldY_in();
                xFieldGoal_in = PathManager.strafeRampReach_in * gamepadStrafe + RobotPose.getFieldX_in();
                aFieldGoal_deg = PathManager.turnRampReach_deg * gamepadTurn * turnSensitivity + lastTurnGoal;
                yFieldDelay_ms = 0;
                xFieldDelay_ms = 0;
                turnFieldDelay_ms = 0;
                lastTurnGoal = aFieldGoal_deg;
                break;
            case P1:
                powerScaling = 0.6;
                yFieldGoal_in = 60;
                xFieldGoal_in = -30;
                aFieldGoal_deg = 0;
                break;
            case P2:
                powerScaling = 0.6;
                yFieldGoal_in = 50;
                xFieldGoal_in = -30;
                aFieldGoal_deg = 0;
                break;
            case P3:
                powerScaling = 0.6;
                yFieldGoal_in = 50;
                xFieldGoal_in = -42;
                aFieldGoal_deg = 0;
                break;
            case BACKBOARD:
                WebCam.streamWebcam(WebCam.WEBCAM.WEBCAM1);
                PathMakerStateMachine.aprilTagDetectionOn = true;
                PathMakerStateMachine.aprilTagDetectionID = 2;
                powerScaling = 0.6;
                // in auto mode the goals are relative to the tag positions (see autoAprilTag)
                yRelativetoTag = 24;
                xRelativeToTag = 0;
                aRelativetoTag = 0;
                break;
            case PIXELSTACKS:
                WebCam.streamWebcam(WebCam.WEBCAM.WEBCAM2);
                PathMakerStateMachine.aprilTagDetectionOn = true;
                PathMakerStateMachine.aprilTagDetectionID = 9;
                powerScaling = 0.6;
                yRelativetoTag = -24;
                xRelativeToTag = 0;
                aRelativetoTag = 0;
                break;

            case DONE:
                break;
        }
    }
    // check for AprilTag detection
    public static void autoAprilTag() {
        PathManager.maxPowerStepUp = 0.1;
        //pathTime_ms = PathManager.timeStep_ms;
        powerScaling = 0.6;
        // need to update with actual robot position
        double distanceToTarget = WebCam.distanceToTarget;
        //double a = WebCam.angleToTarget; // yaw angle
        double a = RobotPose.getFieldA_deg();
        double x = distanceToTarget * Math.sin(Math.toRadians(a));
        double y = -distanceToTarget * Math.cos(Math.toRadians(a));
        double tagXYA[] = RobotPose.rebaseRelativeToTag(y, x, a, PathMakerStateMachine.aprilTagDetectionID);
        // position relative to tags
        yFieldGoal_in = tagXYA[0] - yRelativetoTag; // 12 is the distance from the camera to the front of the robot
        xFieldGoal_in = tagXYA[1] - xRelativeToTag;
        if (Math.abs(a) < 5) {
            xFieldGoal_in += WebCam.offsetToTarget;
        }
        aFieldGoal_deg = tagXYA[2] - aRelativetoTag;
        yFieldDelay_ms = 0;
        xFieldDelay_ms = 0;
        turnFieldDelay_ms = 0;
        lastTurnGoal = aFieldGoal_deg;
        PathManager.forwardRampReach_in = 24;
        PathManager.strafeRampReach_in = 12;
        PathManager.turnRampReach_deg = 45;
        WebCam.stopWebcam();
    }
}