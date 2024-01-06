package org.firstinspires.ftc.teamcode.pathmaker;

import com.qualcomm.robotcore.hardware.Gamepad;

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

    enum GameMode {
        TELEOP, AUTONOMOUS
    }   // end enum GameMode
    enum State {
        INIT, IDLE, DONE, DRIVER_CONTROL,
        AUTO_BACKBOARD, AUTO_START_PATH, AUTO_NEXT_PATH, AUTO_BACKBOARD_ExecutePath, AUTO_ExecutePath
    }   // end enum State

    public static State state;
    static GameMode mode;
    public static boolean aprilTagDetectionOn = false;
    public static int currentPath = -1;
    public static double gamepadForward = 0, gamepadStrafe = 0, gamepadTurn = 0;
    public PathMakerStateMachine() {
        state = State.INIT;
        mode = GameMode.TELEOP;
    }   // end constructor PathMakerStateMachine
    public static void setDriverControlled() {
        state = State.DRIVER_CONTROL;
    }
    public static void setAutonomous() {
        mode = GameMode.AUTONOMOUS;
    }

    public static void updateTele(Gamepad gamepad) throws InterruptedException {
        if (WebCam.detectionAprilTag(583) && gamepad.left_bumper) {
            state = State.AUTO_BACKBOARD;
        } else if (state == State.AUTO_BACKBOARD_ExecutePath) {
            state = State.AUTO_BACKBOARD_ExecutePath;
        } else {
            state = State.DRIVER_CONTROL;
        }
        switch (state) {
            case INIT:
                state = State.IDLE;
                break;
            case DRIVER_CONTROL:
                    gamepadStrafe = gamepad.left_stick_x;
                    gamepadForward = -gamepad.left_stick_y;
                    gamepadTurn = gamepad.right_stick_x;
                    PathDetails.setPath(PathDetails.Path.DriverControlled);
                    PathManager.moveRobot();
                break;
            case AUTO_BACKBOARD:
                PathDetails.setPath(PathDetails.Path.AutoBackboard);
                state = State.AUTO_BACKBOARD_ExecutePath;
                break;
            case AUTO_BACKBOARD_ExecutePath:
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
    }   // end method update

    public static ArrayList<PathDetails.Path> autoPathList;
    private static int nextPath;
    public static void initAuto() {
        state = State.AUTO_START_PATH;
        currentPath = nextPath = 0;
        aprilTagDetectionOn = false;
        mode = GameMode.AUTONOMOUS;
        autoPathList = new ArrayList<PathDetails.Path>();
        autoPathList.add(PathDetails.Path.P1);
        autoPathList.add(PathDetails.Path.P2);
        autoPathList.add(PathDetails.Path.P3);
        autoPathList.add(PathDetails.Path.P4);
    }   // end method initAuto
    public static void updateAuto() throws InterruptedException {
        // process events
        if (aprilTagDetectionOn && WebCam.detectionAprilTag(1) ) {
            state = State.AUTO_BACKBOARD;
            aprilTagDetectionOn = false;
            WebCam.currentDetections = null;
        }
        // process state
        switch (state) {
            case AUTO_START_PATH:
                PathDetails.setPath(autoPathList.get(nextPath));
                state = State.AUTO_ExecutePath;
                break;
            case AUTO_ExecutePath:
                if (PathDetails.elapsedTime_ms.milliseconds() > PathDetails.pathTime_ms) {
                    state = State.AUTO_NEXT_PATH;
                } else if (PathManager.inTargetZone) {
                    state = State.AUTO_NEXT_PATH;
                } else if (PathDetails.elapsedTime_ms.milliseconds()>300 && RobotPose.isRobotAtRest()) { // robot at rest after first moving (300 ms)
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
                    state = State.AUTO_START_PATH;
                }
                break;
            case AUTO_BACKBOARD:
                PathDetails.setPath(PathDetails.Path.AutoBackboard);
                state = State.AUTO_BACKBOARD_ExecutePath;
                break;
            case AUTO_BACKBOARD_ExecutePath:
                if (PathDetails.elapsedTime_ms.milliseconds() > PathDetails.pathTime_ms) {
                    state = State.DONE;
                } else if (PathManager.inTargetZone) {
                    currentPath = nextPath = 0;
                    state = State.AUTO_START_PATH;
                } else if (PathDetails.elapsedTime_ms.milliseconds()>300 && RobotPose.isRobotAtRest()) { // wait until robot first moves (300 ms)
                    state = State.AUTO_START_PATH;
                } else {
                    PathManager.moveRobot();
                }
                break;
            case DONE:
                powerDown();
                break;
            default:
                break;
        }   // end switch (state)
        // assess parallel action and update
        updateParallelAction(ParallelAction.NONE);
    }   // end method update

    private static void powerDown() {
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

    public static void updateParallelAction(ParallelAction parallelAction) {
        switch (parallelAction) {
            case NONE:
                break;
        }   // end switch (parallelAction)
    }   // end method updateParallelAction


}

