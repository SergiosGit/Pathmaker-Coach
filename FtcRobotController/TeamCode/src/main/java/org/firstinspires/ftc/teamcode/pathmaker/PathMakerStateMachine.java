package org.firstinspires.ftc.teamcode.pathmaker;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configuration.RobotConfig;
import org.firstinspires.ftc.teamcode.hw.DriveTrain;
import org.firstinspires.ftc.teamcode.hw.ExtraStuff;
import org.firstinspires.ftc.teamcode.hw.VisionSystem;
import org.firstinspires.ftc.teamcode.op.RobotPose;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.Objects;
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
    public  enum PM_STATE {
        INIT, IDLE, DONE, DRIVER_CONTROL,
        AUTO_BACKBOARD, AUTO_SET_PATH, AUTO_NEXT_PATH, AUTO_APRILTAG_ExecutePath, AUTO_APRILTAG, AUTO_ExecutePath
    }   // end enum State
    public static double telemaxXYPowerStep = 0.3;
    public static PM_STATE pm_state;
    public static CONTROL_MODE control_mode;
    public static boolean autoLaneKeeping = false;
    public static int currentPath = -1, nextPath = -1;
    public static double yPower = 0, yPowerLast = 0;
    public static double xPower = 0, xPowerLast = 0;
    public static double turnPower = 0, turnPowerLast = 0;
    static int xStoppingCounter = 1, yStoppingCounter = 1;
    public static double turnSensitivity = 0.4;
    private static final ElapsedTime switchToAutonomousTimer = new ElapsedTime();
    private static boolean fromManualToAutoHeading = false;
    public static double getAprilTagAngle() {
        List<Integer> aprilIDs = VisionSystem.AprilTag.getAprilTagIDs();
        double AprilTagAngle = 0;
        if (!aprilIDs.isEmpty()) {
            AprilTagDetection tempVision = VisionSystem.AprilTag.getAprilTagByID(aprilIDs.get(0));
            if (tempVision != null){
                double AprilTagX = tempVision.ftcPose.x;
                double AprilTagY = tempVision.ftcPose.y;
                AprilTagAngle = Math.atan(AprilTagX / AprilTagY) * 180 / Math.PI;
            }
        }
        else {
            AprilTagAngle = 0;
        }

        return AprilTagAngle;
    }
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
        RobotPose.readPose();

        switch (pm_state) {
            case INIT:
                pm_state = PM_STATE.IDLE;
                break;
            case DRIVER_CONTROL:
                PathDetails.setPath(PathDetails.Path.DRIVER_CONTROLLED,telemetry);
                getGamepadInput(gamepad);
                // autoLaneKeeping();
                PathManager.moveRobot();
                break;
            default:
                break;
        }   // end switch (state)
    }   // end method updateTele

    private static void getGamepadInput(Gamepad gamepad) {

        if (gamepad.right_trigger > 0.3 && RobotConfig.Camera.useCamera()){
            /*List<Integer> aprilIDs = VisionSystem.AprilTag.getAprilTagIDs();
            if (!aprilIDs.isEmpty()) {
                if (1 < VisionSystem.AprilTag.getAprilTagByID(aprilIDs.get(0)).ftcPose.x) {
                    PathMakerStateMachine.turnPower = ((getAprilTagAngle() / 25) * 0.20);
                }
                else if (-1 > VisionSystem.AprilTag.getAprilTagByID(aprilIDs.get(0)).ftcPose.x) {
                    PathMakerStateMachine.turnPower = ((getAprilTagAngle() / 25) * 0.20);
                }
                else {
                    PathMakerStateMachine.turnPower = 0;
                }
            }*/
        }

        xPower = gamepad.left_stick_x;
        yPower = -gamepad.left_stick_y;
        turnPower = gamepad.right_stick_x * turnSensitivity;

        double gamepadThreshold = 0.1;
        // check if gamepad input is below threshold
        // if so, we will ramp down the input to zero
        if (Math.abs(xPower) < gamepadThreshold) {
            // this ramp doesn't help as much as I wanted
            xPower = (gamepadThreshold / xStoppingCounter++) * Math.signum(xPowerLast);
        } else {
            xStoppingCounter = 1;
        }

        if (Math.abs(yPower) < gamepadThreshold) {
            yPower = (gamepadThreshold / yStoppingCounter++) * Math.signum(yPowerLast);
        } else {
            yStoppingCounter = 1;
        }
        if (Math.abs(turnPower) < gamepadThreshold) {
            turnPower = 0;
        }
        // add ramp for x and y powers
        double maxXYPowerStep = telemaxXYPowerStep;
        double signumXPowerChange = Math.signum(xPower - xPowerLast);
        if (Math.abs(xPower-xPowerLast) > maxXYPowerStep) {
            xPower = xPowerLast + signumXPowerChange * maxXYPowerStep;
            // set minimum power
        }
        if (Math.abs(xPower) < 0.1) {
            xPower *= 0.9;
        }
        double signumYPowerChange = Math.signum(yPower - yPowerLast);
        if (Math.abs(yPower-yPowerLast) > maxXYPowerStep) {
            yPower = yPowerLast + signumYPowerChange * maxXYPowerStep;
            // set minimum power
        }
        if (Math.abs(yPower) < 0.1) {
            yPower *= 0.9;
        }
        // if gamepadTurn is zero, we will automatically keep the robot heading the same
        // if gamepadTurn is not zero, we will turn the robot to the new heading
        PathManager.autonomous_x = false;
        PathManager.autonomous_y = false;
        PathManager.autonomous_a = true;
        if (turnPower == 0) {
            // auto heading
            // keep robot heading the same direction if no turn input
            // this counteracts rotational drifting of the robot
            if (switchToAutonomousTimer.milliseconds() < 200 && fromManualToAutoHeading) {
                // need to wait long enough to get one or heading readings
                // increase 200 to 400 ms for heavy robot (more inertia)
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

        // remember last power
        xPowerLast = xPower;
        yPowerLast = yPower;
        turnPowerLast = turnPower;
    }
    //
    // Autonomous section
    //
    public static void updateAuto(Telemetry telemetry) throws InterruptedException {
        RobotPose.readPose(); // read pose once per auto loop (moved here from PathManager)
        // process state
        switch (pm_state) {
            case AUTO_SET_PATH:
                PathDetails.setPath(PathDetails.autoPathList.get(nextPath),telemetry);
                break;
            case IDLE:
                break;
            case DONE:
                powerDown();
                ExtraStuff.shooter.stop();
                break;
            case AUTO_ExecutePath:
                if (PathDetails.elapsedTime_ms.milliseconds() > PathDetails.pathTime_ms) {
                    pm_state = PM_STATE.AUTO_NEXT_PATH;
                } else if (PathManager.inTargetZone) {
                    pm_state = PM_STATE.AUTO_NEXT_PATH;
                } else if (PathDetails.elapsedTime_ms.milliseconds()>1000 && RobotPose.isRobotAtRest()) { // robot at rest after first moving (1000 ms)
                    // pm_state = PM_STATE.AUTO_NEXT_PATH;
                } else {
                    PathManager.moveRobot();
                    ExtraStuff.Extra_Stuff(PathDetails.targetRpm, PathDetails.intake_power, PathDetails.intake_servo_position);
                }
                break;
            case AUTO_NEXT_PATH:
                setNextPath();
                if (nextPath < 0) {
                    //nextPath = 0;
                    //pm_state = PM_STATE.AUTO_SET_PATH; // repeat from the beginning
                    pm_state = PM_STATE.DONE;
                } else {
                    pm_state = PM_STATE.AUTO_SET_PATH;
                }
                break;
            default:
                break;
        }   // end switch (state)
        ExtraStuff.shooter.update(); // every loop, whatever state the path is in
    }   // end method update
    static void powerDown() {
        DriveTrain.setMotorPowers(0, 0, 0, 0);
    }
    static void powerDown(double minPower) throws InterruptedException {
        DriveTrain.setMotorPowers(minPower, minPower, minPower, minPower);
    }
    private static void setNextPath() {
        nextPath++;
        if (nextPath >= PathDetails.autoPathList.size()) {
            nextPath = -1;
        }
        currentPath = nextPath;
    }   // end method setNextPath
}

