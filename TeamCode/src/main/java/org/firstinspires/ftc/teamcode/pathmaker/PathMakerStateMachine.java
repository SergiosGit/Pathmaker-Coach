package org.firstinspires.ftc.teamcode.pathmaker;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hw.DriveTrain;
import org.firstinspires.ftc.teamcode.hw.WebCam;
import org.firstinspires.ftc.teamcode.op.RobotPose;

import java.util.ArrayList;

public class PathMakerStateMachine {


    public static void terminatePath() {
        // this function needs to be extended to include other options than power down
        // for example, with ramp-reach <= 0, the robot will not stop at the target zone
        // but will continue to move at the maximum velocity. In this case, the robot
        // should be programmed to move to the next target zone.
        powerDown();
    }

    enum ControlMode {
        TELEOP, AUTONOMOUS
    }   // end enum GameMode
    enum State {
        INIT, IDLE, DONE, DRIVER_CONTROL,
        AUTO_BACKBOARD, AUTO_SET_PATH, AUTO_NEXT_PATH, AUTO_APRILTAG_ExecutePath, AUTO_APRILTAG, AUTO_ExecutePath
    }   // end enum State

    public static State state;
    public static ControlMode controlMode;
    public static boolean aprilTagDetectionOn = false;
    public static int aprilTagDetectionID = 0;
    public static int currentPath = -1;
    static double gamepadY = 0;
    static double gamepadX = 0;
    static double gamepadTurn = 0;
    private static double lastGamePadY = 0, lastGamePadX = 0, lastGamePadTurn = 0;
    public static double gamepad_yFieldGoal_in = 0, gamepad_xFieldGoal_in = 0, gamepad_aFieldGoal_deg = 0;
    public PathMakerStateMachine() {
        state = State.INIT;
        controlMode = ControlMode.TELEOP;
    }   // end constructor PathMakerStateMachine
    public static void setDriverControlled() {
        state = State.DRIVER_CONTROL;
    }
    public static void setAutonomous() {
        controlMode = ControlMode.AUTONOMOUS;
    }
    public static void goToNextPath() {
        state = State.AUTO_NEXT_PATH;
    }

    public static void updateTele(Gamepad gamepad, Telemetry telemetry) throws InterruptedException {
        if (aprilTagDetectionOn && WebCam.detectionAprilTag(aprilTagDetectionID, telemetry)  && gamepad.left_bumper) {
            // making sure we don't mistakenly read old data from the WebCam
            // until then the field goals are initialized to 0
            // they will be updated when the robot is re-based in autoAprilTagAndFieldGoals()
            if (WebCam.targetID == aprilTagDetectionID) {
                PathDetails.autoAprilTagAndFieldGoals();
                aprilTagDetectionOn = false;
                WebCam.currentDetections = null;
                state = State.AUTO_APRILTAG_ExecutePath;
                PathDetails.elapsedTime_ms.reset();
            }
        } else if (state == State.AUTO_APRILTAG_ExecutePath) {
            state = State.AUTO_APRILTAG_ExecutePath;
        } else {
            state = State.DRIVER_CONTROL;
        }
        switch (state) {
            case INIT:
                state = State.IDLE;
                break;
            case DRIVER_CONTROL:
                    getGamepadInput(gamepad);
                    PathDetails.setPath(PathDetails.Path.DriverControlled,telemetry);
                    PathManager.moveRobot();
                break;
            case AUTO_BACKBOARD:
                PathDetails.setPath(PathDetails.Path.AutoAprilTag,telemetry);
                state = State.AUTO_APRILTAG_ExecutePath;
                break;
            case AUTO_APRILTAG_ExecutePath:
                if (PathDetails.elapsedTime_ms.milliseconds() > PathDetails.pathTime_ms) {
                    state = State.DRIVER_CONTROL;
                } else if (PathManager.inTargetZone) {
                    state = State.DRIVER_CONTROL;
                } else if (PathDetails.elapsedTime_ms.milliseconds()>300 && RobotPose.isRobotAtRest()) { // wait until robot first moves (300 ms)
                    state = State.DRIVER_CONTROL;
                } else {
                    PathManager.moveRobot();
                }
                break;
            default:
                break;
        }   // end switch (state)
    }   // end method updateTele

    private static void getGamepadInput(Gamepad gamepad) {
        gamepadX = gamepad.left_stick_x;
        gamepadY = -gamepad.left_stick_y;
        gamepadTurn = gamepad.right_stick_x;
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
        gamepad_yFieldGoal_in = PathManager.yRampReach_in * gamepadY;
        gamepad_xFieldGoal_in = PathManager.xRampReach_in * gamepadX;
        gamepad_aFieldGoal_deg = PathManager.turnRampReach_deg * gamepadTurn * PathDetails.turnSensitivity;

    }

    public static ArrayList<PathDetails.Path> autoPathList;
    private static int nextPath;
    public static void initPathList() {
        state = State.AUTO_SET_PATH;
        currentPath = nextPath = 0;
        aprilTagDetectionOn = false;
        controlMode = ControlMode.AUTONOMOUS;
        autoPathList = new ArrayList<PathDetails.Path>();
        autoPathList.add(PathDetails.Path.P1);
        autoPathList.add(PathDetails.Path.BACKBOARD);
        autoPathList.add(PathDetails.Path.P2);
        autoPathList.add(PathDetails.Path.P3);
        autoPathList.add(PathDetails.Path.P4);
        autoPathList.add(PathDetails.Path.PIXELSTACKS);
    }   // end method initAuto
    public static void updateAuto(Telemetry telemetry) throws InterruptedException {
        // process state
        switch (state) {
            case AUTO_SET_PATH:
                PathDetails.setPath(autoPathList.get(nextPath),telemetry);
            case AUTO_APRILTAG:
                if (aprilTagDetectionOn && WebCam.detectionAprilTag(aprilTagDetectionID, telemetry) ) {
                    // making sure we don't mistakenly read old data from the WebCam
                    // until then the field goals are initialized to 0
                    // they will be updated when the robot is re-based in autoAprilTagAndFieldGoals()
                    if (WebCam.targetID == aprilTagDetectionID) {
                        PathDetails.autoAprilTagAndFieldGoals();
                        aprilTagDetectionOn = false;
                        WebCam.currentDetections = null;
                        state = State.AUTO_APRILTAG_ExecutePath;
                        PathDetails.elapsedTime_ms.reset();
                    }
                }
                break;
            case AUTO_APRILTAG_ExecutePath:
                if (PathManager.inTargetZone) {
                    state = State.AUTO_NEXT_PATH;
                } else if (PathDetails.elapsedTime_ms.milliseconds()>500 && RobotPose.isRobotAtRest()) { // wait until robot first moves (300 ms), then check if it rests again
                    state = State.AUTO_NEXT_PATH;
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
                    state = State.AUTO_NEXT_PATH;
                } else if (PathManager.inTargetZone) {
                    state = State.AUTO_NEXT_PATH;
                } else if (PathDetails.elapsedTime_ms.milliseconds()>1000 && RobotPose.isRobotAtRest()) { // robot at rest after first moving (1000 ms)
                    state = State.AUTO_NEXT_PATH;
                } else {
                    PathManager.moveRobot();
                }
                break;
            case AUTO_NEXT_PATH:
                setNextPath();
                if (nextPath < 0) {
                    state = State.DONE;
                } else {
                    state = State.AUTO_SET_PATH;
                }
                break;
        }   // end switch (state)
    }   // end method update

    static void powerDown() {
        DriveTrain.setMotorPowers(0, 0, 0, 0);
    }

    private static void setNextPath() {
        nextPath++;
        if (nextPath >= autoPathList.size()) {
            nextPath = -1;
        }
        currentPath = nextPath;
    }   // end method setNextPath

    public enum ParallelAction {
        NONE
    }   // end enum ParallelAction
}

