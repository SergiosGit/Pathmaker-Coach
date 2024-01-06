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
    public static double forwardGoal_in, forwardOffset_in=0;
    public static double strafeGoal_in, strafeOffset_in=0;
    public static double turnGoal_deg, turnOffset_deg=0;
    // set initial delay
    public static double forwardDelay_ms;
    public static double strafeDelay_ms;
    public static double turnDelay_ms;
    public static double turnSensitivity = 0.4;
    public static ElapsedTime elapsedTime_ms = new ElapsedTime();
    public static double pathTime_ms = 0;


    public static double lastTurnGoal; // RobotPose.getHeadingAngle_deg()

    public static void initializePath() {
        // initialize
        pathTime_ms = 9E9;
        elapsedTime_ms.reset();
        powerScaling = 1;
        forwardGoal_in = 0;
        strafeGoal_in = 0;
        turnGoal_deg = 0;
        forwardDelay_ms = 0;
        strafeDelay_ms = 0;
        turnDelay_ms = 0;
        PathMakerStateMachine.aprilTagDetectionOn = false;
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
    }
    public enum Path {
        P1, P2, P3, P4,
        DriverControlled,
        AutoBackboard, DONE
    }   // end enum Event

    public static void setPath(Path path) {
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
                forwardGoal_in = PathManager.forwardRampReach_in * gamepadForward + RobotPose.getForward_in();
                strafeGoal_in = PathManager.strafeRampReach_in * gamepadStrafe + RobotPose.getStrafe_in();
                turnGoal_deg = PathManager.turnRampReach_deg * gamepadTurn * turnSensitivity + lastTurnGoal;
                forwardDelay_ms = 0;
                strafeDelay_ms = 0;
                turnDelay_ms = 0;
                lastTurnGoal = turnGoal_deg;
                break;
            case AutoBackboard:
                PathManager.maxPowerStepUp = 0.1;
                //pathTime_ms = PathManager.timeStep_ms;
                powerScaling = 0.6;
                // need to update with actual robot position
                double distanceToTarget = WebCam.distanceToTarget;
                double a = WebCam.angleToTarget; // yaw angle
                double x = distanceToTarget * Math.sin(Math.toRadians(a));
                double y = -distanceToTarget * Math.cos(Math.toRadians(a));
                RobotPose.rebase(y, x, a, WebCam.targetID);
                double tagXYA [] = RobotPose.tagOffset(WebCam.targetID);
                forwardGoal_in = tagXYA[0] - 12; // 12 is the distance from the camera to the front of the robot
                strafeGoal_in = tagXYA[1];
                if (Math.abs(a) < 5) {
                    strafeGoal_in += WebCam.offsetToTarget;
                }
                turnGoal_deg = tagXYA[2] + a;
                forwardDelay_ms = 0;
                strafeDelay_ms = 0;
                turnDelay_ms = 0;
                lastTurnGoal = turnGoal_deg;
                PathManager.forwardRampReach_in = 24;
                PathManager.strafeRampReach_in = 12;
                PathManager.turnRampReach_deg = 45;
                break;
            case P1:
                powerScaling = 0.6;
                forwardGoal_in = 60;
                strafeGoal_in = -30;
                turnGoal_deg = 0;
                break;
            case P2:
                powerScaling = 0.6;
                forwardGoal_in = 50;
                strafeGoal_in = -30;
                turnGoal_deg = 0;
                break;
            case P3:
                powerScaling = 0.6;
                forwardGoal_in = 50;
                strafeGoal_in = -42;
                turnGoal_deg = 0;
                break;
            case P4:
                PathMakerStateMachine.aprilTagDetectionOn = true;
                powerScaling = 0.6;
                forwardGoal_in = 60;
                strafeGoal_in = -42;
                turnGoal_deg = 0;
                break;
            case DONE:
                break;
        }

    }
}