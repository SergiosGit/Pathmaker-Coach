package org.firstinspires.ftc.teamcode.pathmaker;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hw.DriveTrain;
import org.firstinspires.ftc.teamcode.hw.WebCam;
import org.firstinspires.ftc.teamcode.op.RobotPose;

import java.util.ArrayList;

public class PathMakerStateMachine {
    // The PathMakerStateMachine manages the state of the robot as defined in the State enum.
    // The PathMakerStateMachine is called from the opmodes Tele_Robot1 and Auto_Robot1.
    // The PathMakerStateMachine calls the PathManager roughly every timeStep_ms.
    // The PathMakerStateMachine has a section for teleop and a section for autonomous.
    // This is the main control loop for the robot. All other states on the robot should be
    // managed by the PathMakerStateMachine. For example, based on the robot field position the
    // arm position should be controlled so that the robot does not interfere with the truss.

    enum CONTROL_MODE {
        TELEOP, AUTONOMOUS
    }   // end enum GameMode
    enum PM_STATE {
        INIT, IDLE, DONE, DRIVER_CONTROL,
        AUTO_BACKBOARD, AUTO_SET_PATH, AUTO_NEXT_PATH, AUTO_APRILTAG_ExecutePath, AUTO_APRILTAG, AUTO_ExecutePath
    }   // end enum State

    public static PM_STATE pm_state;
    public static CONTROL_MODE control_mode;
    public static boolean aprilTagDetectionOn = false;
    public static int aprilTagDetectionID = 0;
    public static int currentPath = -1, nextPath = -1;
    static double gamepadY = 0;
    static double gamepadX = 0;
    static double gamepadTurn = 0;
    public static double turnSensitivity = 0.4;
    private static ElapsedTime switchToAutonomousTimer = new ElapsedTime();
    private static boolean fromManualToAutoHeading = false;

    public PathMakerStateMachine() {
        pm_state = PM_STATE.INIT;
        control_mode = CONTROL_MODE.TELEOP;
    }   // end constructor PathMakerStateMachine
    public static void setDriverControlled() {
        pm_state = PM_STATE.DRIVER_CONTROL;
        PathManager.autonomous_x = false;
        PathManager.autonomous_y = false;
        PathManager.autonomous_a = false;
    }
    public static void setAutonomous() {
        control_mode = CONTROL_MODE.AUTONOMOUS;
        PathManager.autonomous_x = true;
        PathManager.autonomous_y = true;
        PathManager.autonomous_a = true;
    }
    //
    // Teleop section
    //
    public static void updateTele(Gamepad gamepad, Telemetry telemetry) throws InterruptedException {
        if (aprilTagDetectionOn && WebCam.detectionAprilTag(aprilTagDetectionID, telemetry)  && gamepad.left_bumper) {
            // making sure we don't mistakenly read old data from the WebCam
            // until then the field goals are initialized to 0
            // they will be updated when the robot is re-based in autoAprilTagAndFieldGoals()
            if (WebCam.targetID == aprilTagDetectionID) {
                PathDetails.autoAprilTagAndFieldGoals();
                aprilTagDetectionOn = false;
                WebCam.currentDetections = null;
                pm_state = PM_STATE.AUTO_APRILTAG_ExecutePath;
                PathDetails.elapsedTime_ms.reset();
            }
        } else if (pm_state == PM_STATE.AUTO_APRILTAG_ExecutePath) {
            pm_state = PM_STATE.AUTO_APRILTAG_ExecutePath;
        } else {
            pm_state = PM_STATE.DRIVER_CONTROL;
        }
        switch (pm_state) {
            case INIT:
                pm_state = PM_STATE.IDLE;
                break;
            case DRIVER_CONTROL:
                PathDetails.setPath(PathDetails.Path.DRIVER_CONTROLLED,telemetry);
                getGamepadInput(gamepad);
                autoLaneKeeping();
                PathManager.moveRobot();
                break;
            case AUTO_BACKBOARD:
                PathDetails.setPath(PathDetails.Path.AUTO_APRIL_TAG,telemetry);
                pm_state = PM_STATE.AUTO_APRILTAG_ExecutePath;
                break;
            case AUTO_APRILTAG_ExecutePath:
                if (PathDetails.elapsedTime_ms.milliseconds() > PathDetails.pathTime_ms) {
                    pm_state = PM_STATE.DRIVER_CONTROL;
                } else if (PathManager.inTargetZone) {
                    pm_state = PM_STATE.DRIVER_CONTROL;
                } else if (PathDetails.elapsedTime_ms.milliseconds()>300 && RobotPose.isRobotAtRest()) { // wait until robot first moves (300 ms)
                    pm_state = PM_STATE.DRIVER_CONTROL;
                } else {
                    PathManager.moveRobot();
                }
                break;
            default:
                break;
        }   // end switch (state)
    }   // end method updateTele
    private static void autoLaneKeeping() {
        // auto lane keeping while crossing the trusses in driver control mode
        if (RobotPose.getFieldY_in() > 27) {
            // if robot is not in target zone, we will move it back
            PathDetails.xFieldGoal_in = -24;
            PathManager.autonomous_x = true;
        } else {
            PathManager.autonomous_x = false;
        }
    }
    private static void getGamepadInput(Gamepad gamepad) {
        gamepadX = gamepad.left_stick_x;
        gamepadY = -gamepad.left_stick_y;
        gamepadTurn = gamepad.right_stick_x * turnSensitivity;

        double gamepadThreshold = 0.02;
        // check if gampad input is below threshold
        // if so, we will ramp down the input to zero
        if (Math.abs(gamepadX) < gamepadThreshold) {
            gamepadX = 0;
        }
        if (Math.abs(gamepadY) < gamepadThreshold) {
            gamepadY = 0;
        }
        if (Math.abs(gamepadTurn) < gamepadThreshold) {
            gamepadTurn = 0;
        }
        // if gamepadTurn is zero, we will automatically keep the robot heading the same
        // if gamepadTurn is not zero, we will turn the robot to the new heading
        PathManager.autonomous_x = false;
        PathManager.autonomous_y = false;
        PathManager.autonomous_a = true;
        if (gamepadTurn == 0) {
            // auto heading
            // keep robot heading the same direction if no turn input
            // this counteracts rotational drifting of the robot
            if (switchToAutonomousTimer.milliseconds() < 200 && fromManualToAutoHeading) {
                // need to wait long enough to get one or two IMU readings
                // before switching to autonomous mode to avoid bounce back
                PathDetails.aFieldGoal_deg = RobotPose.getFieldAngle_deg();
            } else {
                PathManager.autonomous_a = true;
                fromManualToAutoHeading = false;
            }
        } else {
            // manual heading
            // keep track of the last actual robot heading
            // this is used when switching from driver control to autonomous for turning
            fromManualToAutoHeading = true;
            switchToAutonomousTimer.reset();
            PathManager.autonomous_a = false;
        }
    }
    //
    // Autonomous section
    //
    public static void updateAuto(Telemetry telemetry) throws InterruptedException {
        // process state
        switch (pm_state) {
            case AUTO_SET_PATH:
                PathDetails.setPath(PathDetails.autoPathList.get(nextPath),telemetry);
            case AUTO_APRILTAG:
                powerDown(); // making sure we are stopped for AprilTag detection
                if (aprilTagDetectionOn && WebCam.detectionAprilTag(aprilTagDetectionID, telemetry) ) {
                    // making sure we don't mistakenly read old data from the WebCam
                    // until then the field goals are initialized to 0
                    // they will be updated when the robot is re-based in autoAprilTagAndFieldGoals()
                    if (WebCam.targetID == aprilTagDetectionID) {
                        PathDetails.autoAprilTagAndFieldGoals();
                        aprilTagDetectionOn = false;
                        WebCam.currentDetections = null;
                        pm_state = PM_STATE.AUTO_APRILTAG_ExecutePath;
                        PathDetails.elapsedTime_ms.reset();
                    }
                }
                break;
            case AUTO_APRILTAG_ExecutePath:
                if (PathManager.inTargetZone) {
                    pm_state = PM_STATE.AUTO_NEXT_PATH;
                } else if (PathDetails.elapsedTime_ms.milliseconds()>500 && RobotPose.isRobotAtRest()) { // wait until robot first moves (300 ms), then check if it rests again
                    pm_state = PM_STATE.AUTO_NEXT_PATH;
                } else {
                    PathManager.moveRobot();
                }
                break;
            case IDLE:
                break;
            case DONE:
                powerDown();
                break;
            default:
                break;
            case AUTO_ExecutePath:
                if (PathDetails.elapsedTime_ms.milliseconds() > PathDetails.pathTime_ms) {
                    pm_state = PM_STATE.AUTO_NEXT_PATH;
                } else if (PathManager.inTargetZone) {
                    pm_state = PM_STATE.AUTO_NEXT_PATH;
                } else if (PathDetails.elapsedTime_ms.milliseconds()>1000 && RobotPose.isRobotAtRest()) { // robot at rest after first moving (1000 ms)
                    pm_state = PM_STATE.AUTO_NEXT_PATH;
                } else {
                    PathManager.moveRobot();
                }
                break;
            case AUTO_NEXT_PATH:
                setNextPath();
                if (nextPath < 0) {
                    pm_state = PM_STATE.DONE;
                } else {
                    pm_state = PM_STATE.AUTO_SET_PATH;
                }
                break;
        }   // end switch (state)
    }   // end method update
    static void powerDown() {
        DriveTrain.setMotorPowers(0, 0, 0, 0);
    }
    private static void setNextPath() {
        nextPath++;
        if (nextPath >= PathDetails.autoPathList.size()) {
            nextPath = -1;
        }
        currentPath = nextPath;
    }   // end method setNextPath
}

