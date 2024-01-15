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
    public static double xInitialPowerSignum = 1, yInitialPowerSignum = 1, turnInitialPowerSignum = 1;
    // set initial delay
    public static double yFieldDelay_ms;
    public static double xFieldDelay_ms;
    public static double turnFieldDelay_ms;
    public static double turnSensitivity = 0.4;
    public static ElapsedTime elapsedTime_ms = new ElapsedTime();
    public static double pathTime_ms = 0;
    public  static PathMakerStateMachine.State PMSMstate;
    private static WebCam.WEBCAM currentWebCam = WebCam.WEBCAM.NONE;

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
        PathManager.yTargetZone_in = 1;
        PathManager.xTargetZone_in = 1;
        PathManager.turnTargetZone_deg = 1;
        PathManager.yRampReach_in = 11;
        PathManager.xRampReach_in = 11;
        PathManager.turnRampReach_deg = 45;
        PathManager.yMinVelocity_inPerSec = 2;
        PathManager.xMinVelocity_inPerSec = 2;
        PathManager.turnMinVelocity_degPerSec = 4;
        WebCam.stopWebcam();
    }
    public enum Path {
        P1, P2, P3, BACKBOARD,
        DriverControlled,
        AutoAprilTag, PIXELSTACKS, P4, DONE
    }   // end enum Event

    public static void setPath(Path path, Telemetry telemetry) throws InterruptedException {
        initializePath();
        switch (path) {
            case DriverControlled:
                PathMakerStateMachine.controlMode = PathMakerStateMachine.ControlMode.TELEOP;
                PathManager.maxPowerStepUp = 0.1;
                //pathTime_ms = PathManager.timeStep_ms;
                powerScaling = 1;
                // need to update with actual robot position
                //robotAngle = RobotPose.getHeadingAngle_deg();
                // Note that "forward" is directly from the gamepad, i.e. a value between -1 and 1.
                // Robot forward power is maximum for requested gaols larger or equal than forwardRampReach_in (this is controlled by the PathManager)
                // Similar for strafe and turn.
                yFieldGoal_in = PathMakerStateMachine.gamepad_yFieldGoal_in + RobotPose.getFieldY_in();
                xFieldGoal_in = PathMakerStateMachine.gamepad_xFieldGoal_in + RobotPose.getFieldX_in();
                aFieldGoal_deg = PathMakerStateMachine.gamepad_aFieldGoal_deg * turnSensitivity + lastTurnGoal;
                yFieldDelay_ms = 0;
                xFieldDelay_ms = 0;
                turnFieldDelay_ms = 0;
                lastTurnGoal = aFieldGoal_deg;
                break;
            case P1:
                PathMakerStateMachine.controlMode = PathMakerStateMachine.ControlMode.AUTONOMOUS;
                PathMakerStateMachine.state = PathMakerStateMachine.State.AUTO_ExecutePath;
                powerScaling = 0.5;
                xFieldGoal_in = -36; yFieldGoal_in = 48; aFieldGoal_deg = 0;
                calculateInitialPowerSignum(xFieldGoal_in, yFieldGoal_in, aFieldGoal_deg);
                break;
            case P2:
                PathMakerStateMachine.controlMode = PathMakerStateMachine.ControlMode.AUTONOMOUS;
                PathMakerStateMachine.state = PathMakerStateMachine.State.AUTO_ExecutePath;
                PathManager.yRampReach_in = 0;
                powerScaling = 0.5;
                xFieldGoal_in = -12; yFieldGoal_in = 12; aFieldGoal_deg = 0;
                calculateInitialPowerSignum(xFieldGoal_in, yFieldGoal_in, aFieldGoal_deg);
                break;
            case P3:
                PathMakerStateMachine.controlMode = PathMakerStateMachine.ControlMode.AUTONOMOUS;
                PathMakerStateMachine.state = PathMakerStateMachine.State.AUTO_ExecutePath;
                PathManager.yRampReach_in = 0;
                powerScaling = 0.5;
                xFieldGoal_in = -12; yFieldGoal_in = -6; aFieldGoal_deg = 0;
                calculateInitialPowerSignum(xFieldGoal_in, yFieldGoal_in, aFieldGoal_deg);
                break;
            case P4:
                PathMakerStateMachine.controlMode = PathMakerStateMachine.ControlMode.AUTONOMOUS;
                PathMakerStateMachine.state = PathMakerStateMachine.State.AUTO_ExecutePath;
                PathManager.yRampReach_in = 5;
                powerScaling = 0.5;
                xFieldGoal_in = -12; yFieldGoal_in = -20; aFieldGoal_deg = 0;
                break;
            case BACKBOARD:
                PathMakerStateMachine.controlMode = PathMakerStateMachine.ControlMode.AUTONOMOUS;
                PathMakerStateMachine.state = PathMakerStateMachine.State.AUTO_APRILTAG;
                currentWebCam = WebCam.WEBCAM.WEBCAM1;
                WebCam.streamWebcam(currentWebCam, telemetry);
                PathMakerStateMachine.aprilTagDetectionOn = true;
                PathMakerStateMachine.aprilTagDetectionID = 2;
                powerScaling = 0.5;
                // in auto mode the goals are relative to the tag positions (see autoAprilTag)
                yRelativetoTag = -10;
                xRelativeToTag = 0;
                aRelativetoTag = 0;
                break;
            case PIXELSTACKS:
                PathMakerStateMachine.controlMode = PathMakerStateMachine.ControlMode.AUTONOMOUS;
                PathMakerStateMachine.state = PathMakerStateMachine.State.AUTO_APRILTAG;
                currentWebCam = WebCam.WEBCAM.WEBCAM2;
                WebCam.streamWebcam(currentWebCam, telemetry);
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
        xInitialPowerSignum = Math.signum( goalX - RobotPose.getFieldX_in());
        yInitialPowerSignum = Math.signum( goalY - RobotPose.getFieldY_in());
        turnInitialPowerSignum = Math.signum( goalA - RobotPose.getFieldAngle_deg());
    }

    // check for AprilTag detection
    public static void autoAprilTagAndFieldGoals() {
        PathMakerStateMachine.powerDown(); // just to make sure we are stopped
        int xyWebCamMultiplier = 1;
        if (currentWebCam == WebCam.WEBCAM.WEBCAM2) {
            xyWebCamMultiplier = -1;
        }
        PathManager.maxPowerStepUp = 0.1;
        //pathTime_ms = PathManager.timeStep_ms;
        powerScaling = 0.6;
        // need to update with actual robot position
        double distanceToTarget = WebCam.distanceToTarget;
        //double a = WebCam.angleToTarget; // yaw angle
        double a = RobotPose.getFieldAngle_deg();
        double x = -distanceToTarget * Math.sin(Math.toRadians(a));
        // limit sideways motion to 5 inches
        if (Math.abs(x) > 5) {
            x = 5 * Math.signum(x);
        }
        double y = distanceToTarget * Math.cos(Math.toRadians(a)) * xyWebCamMultiplier;
        double tagXYA[] = RobotPose.rebaseRelativeToTag(x, y, a, PathMakerStateMachine.aprilTagDetectionID);
        // position relative to tags
        yFieldGoal_in = tagXYA[0] + yRelativetoTag;
        xFieldGoal_in = tagXYA[1] + xRelativeToTag;
        if (Math.abs(a) < 5) {
            xFieldGoal_in += WebCam.offsetToTarget;
        }
        aFieldGoal_deg = tagXYA[2] + aRelativetoTag;
        yFieldDelay_ms = 0;
        xFieldDelay_ms = 0;
        turnFieldDelay_ms = 0;
        lastTurnGoal = aFieldGoal_deg;
        PathManager.yRampReach_in = 24;
        PathManager.xRampReach_in = 12;
        PathManager.turnRampReach_deg = 45;
        WebCam.stopWebcam();
    }
}