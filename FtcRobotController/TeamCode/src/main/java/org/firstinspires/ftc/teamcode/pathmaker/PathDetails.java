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

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configuration.ShooterConfig;
import org.firstinspires.ftc.teamcode.op.RobotPose;

import java.util.ArrayList;

@Configurable
public class PathDetails {
    // for testing
    public static double  p1maxPowerStepUp = 0.05;
    public static double  p1powerScaling = 1;
    public static double  p1xFieldGoal_in = 0;
    public static double  p1yFieldGoal_in = 45;
    public static double  p1aFieldGoal_deg = 0;
    public static double  p1pathTime_ms = 9E9;
    public static double  p1yFieldDelay_ms = 0;
    public static double  p1xFieldDelay_ms = 0;
    public static double  p1turnFieldDelay_ms = 0;
    public static double  p1xRampReach_in = 10;
    public static double  p1yRampReach_in = 15;
    public static double  p1turnRampReach_deg = 45;
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
    public static double xInitialPowerSignum = 1, yInitialPowerSignum = 1, aInitialPowerSignum = 1;
    // set initial delay
    public static double yFieldDelay_ms;
    public static double xFieldDelay_ms;
    public static double turnFieldDelay_ms;
    public static ElapsedTime elapsedTime_ms = new ElapsedTime();
    public static double pathTime_ms = 0;
    public  static PathMakerStateMachine.PM_STATE PMSMstate;
    public static double roombaRadius = 200;

    public static boolean aprilTagNavigation = false;
    public static boolean rebaseUsingAprilTag = false; // tag must be in view
    public static int aprilTagID = 10;

    public static double lastTurnGoal;
    public static int currentRotateDeg = 0; // Store the current rotation degree for telemetry
    public static double intake_power = 0;
    public static double targetRpm = 0; // wheel speed in RPM
    public static double intake_servo_position = 0;
    public enum AutoType {
        RedClose, RedFar,
        BlueClose, BlueFar,
        Test,
    }
    public static AutoType thisAutoStrategy = AutoType.Test;
    public static void initializePath() {

        PathManager.rebaseSuccessful = false;
        aprilTagNavigation = false;
        rebaseUsingAprilTag = false;
        // initialize each new path
        pathTime_ms = 9E9;
        elapsedTime_ms.reset();
        powerScaling = 1;
        yFieldDelay_ms = 0;
        xFieldDelay_ms = 0;
        turnFieldDelay_ms = 0;
        PMSMstate = PathMakerStateMachine.PM_STATE.AUTO_SET_PATH;
        PathManager.powerScalingTurn = 1;
        PathManager.maxPowerStepUp = p1maxPowerStepUp;
        PathManager.inTargetZone = false;
        PathManager.yTargetZone_in = 1;
        PathManager.xTargetZone_in = 1;
        PathManager.turnTargetZone_deg = 10;
        PathManager.yRampReach_in = 5;
        PathManager.xRampReach_in = 4;
        PathManager.turnRampReach_deg = 30;
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
        PathManager.yPowerWeight = 1;
        PathManager.xPowerWeight = 1;
        PathManager.turnPowerWeight = 1;
    }
    public enum Path {
        // Each path is labeled with a name as defined in this enum
        R11, R12, R13, R14, R15, R16, R17, R18, R19, R10,
        R21, R22, R23, R24, R25, R26, R27, R28, R29, R20,
        B11, B12, B13, B14, B15, B16, B17, B18, B19, B10,
        B21, B22, B23, B24, B25, B26, B27, B28, B29, B20,

        DRIVER_CONTROLLED,
        DONE
    }   // end enum Event
    public static ArrayList<Path> autoPathList;
    public static void initAutoPathList() {
        PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_SET_PATH;
        PathMakerStateMachine.currentPath = PathMakerStateMachine.nextPath = 0;
        PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
        autoPathList = new ArrayList<PathDetails.Path>();
        if (thisAutoStrategy == AutoType.RedClose) {
            autoPathList.add(PathDetails.Path.R11);
            autoPathList.add(PathDetails.Path.R12);
            autoPathList.add(PathDetails.Path.R13);
            autoPathList.add(PathDetails.Path.R14);
            autoPathList.add(PathDetails.Path.R15);
            autoPathList.add(PathDetails.Path.R16);
        }
        else if (thisAutoStrategy == AutoType.RedFar) {
            autoPathList.add(PathDetails.Path.R21);
            autoPathList.add(PathDetails.Path.R22);
            autoPathList.add(PathDetails.Path.R23);
            autoPathList.add(PathDetails.Path.R24);
            autoPathList.add(PathDetails.Path.R25);
            autoPathList.add(PathDetails.Path.R26);
            autoPathList.add(PathDetails.Path.R27);
            autoPathList.add(PathDetails.Path.R28);
            autoPathList.add(PathDetails.Path.R29);
        }
        else if (thisAutoStrategy == AutoType.BlueClose) {
            autoPathList.add(PathDetails.Path.B11);
            autoPathList.add(PathDetails.Path.B12);
            autoPathList.add(PathDetails.Path.B13);
            autoPathList.add(PathDetails.Path.B14);
            autoPathList.add(PathDetails.Path.B15);
        }
        else if (thisAutoStrategy == AutoType.BlueFar) {
            autoPathList.add(PathDetails.Path.B21);
            autoPathList.add(PathDetails.Path.B22);
            autoPathList.add(PathDetails.Path.B20);
            autoPathList.add(PathDetails.Path.B23);
            autoPathList.add(PathDetails.Path.B24);
            autoPathList.add(PathDetails.Path.B25);
            autoPathList.add(PathDetails.Path.B26);
            autoPathList.add(PathDetails.Path.B27);
            autoPathList.add(PathDetails.Path.B28);
        }
        else if (thisAutoStrategy == AutoType.Test) {
            autoPathList.add(PathDetails.Path.DONE);
        }
        else {
            //autoPathList.add(new PathInstruction("Test", 0, 20, 0, 1, 0, 0, 0, 7000, 0, 0, 0, 0));
        }
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
                powerScaling = 1;
                PathManager.turnPower = 0.5;
                break;
            case R11:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = -4;
                yFieldGoal_in = 0;
                aFieldGoal_deg = 0;
                powerScaling = 0.3;
                xFieldDelay_ms = 0;
                yFieldDelay_ms = 0;
                intake_power = 0;
                intake_servo_position = 1;
                targetRpm = ShooterConfig.autoTargetRpm;
                pathTime_ms = 1500;
                break;
            case R12:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = -4;
                yFieldGoal_in = -11;
                aFieldGoal_deg = 0;
                powerScaling = 0.3;
                xFieldDelay_ms = 0;
                yFieldDelay_ms = 0;
                intake_power = 0;
                intake_servo_position = 1;
                targetRpm = ShooterConfig.autoTargetRpm;
                pathTime_ms = 7000;
                break;
            case R13:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = -4;
                yFieldGoal_in = -100;
                aFieldGoal_deg = 0;
                powerScaling = 0.3;
                xFieldDelay_ms = 10000;
                yFieldDelay_ms = 10000;
                intake_power = 0;
                intake_servo_position = 0;
                targetRpm = ShooterConfig.autoTargetRpm;
                pathTime_ms = 5000;
                break;
            case R14:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = -4;
                yFieldGoal_in = -11;
                aFieldGoal_deg = -85;
                powerScaling = 0.5;
                xFieldDelay_ms = 500000;
                yFieldDelay_ms = 500000;
                intake_power = 0;
                intake_servo_position = 1;
                targetRpm = 0;
                pathTime_ms = 3000;
                break;
            case R15:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = -44;
                yFieldGoal_in = 14;
                aFieldGoal_deg = -85;
                powerScaling = 0.5;
                xFieldDelay_ms = 0;
                yFieldDelay_ms = 0;
                intake_power = 0;
                intake_servo_position = 1;
                targetRpm = 0;
                pathTime_ms = 1200;
                break;
            case R16:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = -50;
                yFieldGoal_in = 14;
                aFieldGoal_deg = -85;
                powerScaling = 0.2;
                xFieldDelay_ms = 0;
                yFieldDelay_ms = 0;
                intake_power = 0;
                intake_servo_position = 1;
                targetRpm = 0;
                pathTime_ms = 7000;
                break;
            case B11:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = -4;
                yFieldGoal_in = 0;
                aFieldGoal_deg = 0;
                powerScaling = 0.3;
                xFieldDelay_ms = 0;
                yFieldDelay_ms = 0;
                intake_power = 0;
                intake_servo_position = 1;
                targetRpm = ShooterConfig.autoTargetRpm;
                pathTime_ms = 1500;
                break;
            case B12:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = -4;
                yFieldGoal_in = -11;
                aFieldGoal_deg = 0;
                powerScaling = 0.3;
                xFieldDelay_ms = 0;
                yFieldDelay_ms = 0;
                intake_power = 0;
                intake_servo_position = 1;
                targetRpm = ShooterConfig.autoTargetRpm;
                pathTime_ms = 7000;
                break;
            case B13:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = -4;
                yFieldGoal_in = -1100;
                aFieldGoal_deg = 0;
                powerScaling = 0.3;
                xFieldDelay_ms = 10000;
                yFieldDelay_ms = 10000;
                intake_power = 0;
                intake_servo_position = 0;
                targetRpm = ShooterConfig.autoTargetRpm;
                pathTime_ms = 5000;
                break;
            case B14:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = -4;
                yFieldGoal_in = -11;
                aFieldGoal_deg = -85;
                powerScaling = 0.5;
                xFieldDelay_ms = 500000;
                yFieldDelay_ms = 500000;
                intake_power = 0;
                intake_servo_position = 1;
                targetRpm = 0;
                pathTime_ms = 3000;
                break;
            case B15:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = -44;
                yFieldGoal_in = 14;
                aFieldGoal_deg = -85;
                powerScaling = 0.5;
                xFieldDelay_ms = 0;
                yFieldDelay_ms = 0;
                intake_power = 0;
                intake_servo_position = 1;
                targetRpm = 0;
                pathTime_ms = 7000;
                break;
            case B21:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = 0;
                yFieldGoal_in = -11;
                aFieldGoal_deg = 0;
                powerScaling = 0.5;
                xFieldDelay_ms = 0;
                yFieldDelay_ms = 0;
                intake_power = 0;
                intake_servo_position = 1;
                targetRpm = ShooterConfig.autoTargetRpm;
                pathTime_ms = 2000;
                break;
            case B22:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = -10;
                yFieldGoal_in = -11;
                aFieldGoal_deg = 0;
                powerScaling = 0.5;
                xFieldDelay_ms = 0;
                yFieldDelay_ms = 0;
                intake_power = 0;
                intake_servo_position = 1;
                targetRpm = ShooterConfig.autoTargetRpm;
                pathTime_ms = 2000;
                break;
            case B20:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = -10;
                yFieldGoal_in = -1100;
                aFieldGoal_deg = 0;
                powerScaling = 0.5;
                xFieldDelay_ms = 10000;
                yFieldDelay_ms = 10000;
                intake_power = 0;
                intake_servo_position = 0;
                targetRpm = ShooterConfig.autoTargetRpm;
                pathTime_ms = 5000;
                break;
            case B23:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = -10;
                yFieldGoal_in = 31;
                aFieldGoal_deg = 85;
                powerScaling = 0.5;
                xFieldDelay_ms = 1000000;
                yFieldDelay_ms = 1000000;
                intake_power = 0;
                intake_servo_position = 1;
                targetRpm = 0;
                pathTime_ms = 4000;
                break;
            case B24:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = -10;
                yFieldGoal_in = 21;
                aFieldGoal_deg = 85;
                powerScaling = 0.5;
                xFieldDelay_ms = 0;
                yFieldDelay_ms = 0;
                intake_power = 0;
                intake_servo_position = 1;
                targetRpm = 0;
                pathTime_ms = 2000;
                break;
            case B25:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = 8;
                yFieldGoal_in = 21;
                aFieldGoal_deg = 85;
                powerScaling = 0.3;
                xFieldDelay_ms = 0;
                yFieldDelay_ms = 0;
                intake_power = 0;
                intake_servo_position = 1;
                targetRpm = 0;
                pathTime_ms = 4000;
                break;
            case B26:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = 8;
                yFieldGoal_in = 50;
                aFieldGoal_deg = 85;
                powerScaling = 0.5;
                xFieldDelay_ms = 0;
                yFieldDelay_ms = 0;
                intake_power = 0;
                intake_servo_position = 1;
                targetRpm = 0;
                pathTime_ms = 4000;
                break;
            case B27:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = 10;
                yFieldGoal_in = 50;
                aFieldGoal_deg = 85;
                powerScaling = 0.3;
                xFieldDelay_ms = 0;
                yFieldDelay_ms = 0;
                intake_power = 0;
                intake_servo_position = 1;
                targetRpm = 0;
                pathTime_ms = 4000;
                break;
            case B28:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = 10;
                yFieldGoal_in = 82;
                aFieldGoal_deg = 85;
                powerScaling = 0.3;
                xFieldDelay_ms = 0;
                yFieldDelay_ms = 0;
                intake_power = 0;
                intake_servo_position = 1;
                targetRpm = 0;
                pathTime_ms = 4000;
                break;
            case R21:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = 0;
                yFieldGoal_in = -11;
                aFieldGoal_deg = 0;
                powerScaling = 0.5;
                xFieldDelay_ms = 0;
                yFieldDelay_ms = 0;
                intake_power = 0;
                intake_servo_position = 1;
                targetRpm = ShooterConfig.autoTargetRpm;
                pathTime_ms = 2000;
                break;
            case R22:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = -10;
                yFieldGoal_in = -11;
                aFieldGoal_deg = 0;
                powerScaling = 0.5;
                xFieldDelay_ms = 0;
                yFieldDelay_ms = 0;
                intake_power = 0;
                intake_servo_position = 1;
                targetRpm = ShooterConfig.autoTargetRpm;
                pathTime_ms = 2000;
                break;
            case R20:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = -10;
                yFieldGoal_in = -1100;
                aFieldGoal_deg = 0;
                powerScaling = 0.5;
                xFieldDelay_ms = 10000;
                yFieldDelay_ms = 10000;
                intake_power = 0;
                intake_servo_position = 0;
                targetRpm = ShooterConfig.autoTargetRpm;
                pathTime_ms = 5000;
                break;
            case R23:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = -10;
                yFieldGoal_in = 31;
                aFieldGoal_deg = -85;
                powerScaling = 0.5;
                xFieldDelay_ms = 1000000;
                yFieldDelay_ms = 1000000;
                intake_power = 0;
                intake_servo_position = 1;
                targetRpm = 0;
                pathTime_ms = 4000;
                break;
            case R24:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = -10;
                yFieldGoal_in = 21;
                aFieldGoal_deg = -85;
                powerScaling = 0.5;
                xFieldDelay_ms = 0;
                yFieldDelay_ms = 0;
                intake_power = 0;
                intake_servo_position = 1;
                targetRpm = 0;
                pathTime_ms = 2000;
                break;
            case R25:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = 28;
                yFieldGoal_in = 21;
                aFieldGoal_deg = -85;
                powerScaling = 0.3;
                xFieldDelay_ms = 0;
                yFieldDelay_ms = 0;
                intake_power = 0;
                intake_servo_position = 1;
                targetRpm = 0;
                pathTime_ms = 4000;
                break;
            case R26:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = 28;
                yFieldGoal_in = 50;
                aFieldGoal_deg = -85;
                powerScaling = 0.5;
                xFieldDelay_ms = 0;
                yFieldDelay_ms = 0;
                intake_power = 0;
                intake_servo_position = 1;
                targetRpm = 0;
                pathTime_ms = 4000;
                break;
            case R27:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = 30;
                yFieldGoal_in = 50;
                aFieldGoal_deg = -85;
                powerScaling = 0.3;
                xFieldDelay_ms = 0;
                yFieldDelay_ms = 0;
                intake_power = 0;
                intake_servo_position = 1;
                targetRpm = 0;
                pathTime_ms = 4000;
                break;
            case R28:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = 30;
                yFieldGoal_in = 82;
                aFieldGoal_deg = -85;
                powerScaling = 0.3;
                xFieldDelay_ms = 0;
                yFieldDelay_ms = 0;
                intake_power = 0;
                intake_servo_position = 1;
                targetRpm = 0;
                pathTime_ms = 4000;
                break;
            case R29:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_ExecutePath;
                xFieldGoal_in = 35;
                yFieldGoal_in = 82;
                aFieldGoal_deg = -85;
                powerScaling = 0.2;
                xFieldDelay_ms = 0;
                yFieldDelay_ms = 0;
                intake_power = 0;
                intake_servo_position = 1;
                targetRpm = 0;
                pathTime_ms = 4000;
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
    
    /**
     * Get current path goal information for telemetry display.
     * @return Array containing [rotateDeg, yFieldGoal_in, xFieldGoal_in]
     */
    public static double[] getCurrentPathGoals() {
        return new double[]{currentRotateDeg, yFieldGoal_in, xFieldGoal_in};
    }
    
    /**
     * Get current rotation degree for telemetry.
     * @return Current rotation degree (0-359)
     */
    public static int getCurrentRotateDeg() {
        return currentRotateDeg;
    }
    
    /**
     * Update field goals for telemetry display.
     * In teleop mode, this can be used to show current targets or robot position.
     */
    public static void updateFieldGoalsForTelemetry() {
        // For telemetry display, we can set goals based on current robot position
        // or use them to show intended movement direction
        if (PathMakerStateMachine.control_mode == PathMakerStateMachine.CONTROL_MODE.TELEOP) {
            // In teleop mode, set goals to current robot position for reference
            yFieldGoal_in = RobotPose.getFieldY_in();
            xFieldGoal_in = RobotPose.getFieldX_in();
            aFieldGoal_deg = RobotPose.getFieldAngle_deg();
        }
        // In autonomous mode, goals are set by the path system
    }
    
    /**
     * Set field goals to a specific position (useful for testing).
     */
    public static void setFieldGoals(double yGoal, double xGoal, double aGoal) {
        yFieldGoal_in = yGoal;
        xFieldGoal_in = xGoal;
        aFieldGoal_deg = aGoal;
    }
}