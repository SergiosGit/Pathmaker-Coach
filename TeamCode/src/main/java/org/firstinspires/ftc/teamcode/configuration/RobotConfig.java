package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * Centralized configuration for robot parameters.
 * This class consolidates all robot-specific settings for easy portability
 * between different robot configurations and game layouts.
 */
@Config
public class RobotConfig {
    
    // ===== ROBOT PHYSICAL PARAMETERS =====
    public static class Physical {
        public static double robotMassKg = 10.0;
        public static double wheelDiameterInches = 4.0;
        public static double wheelCircumferenceInches = wheelDiameterInches * Math.PI;
        public static double encoderTicksPerRevolution = 537.6; // REV HD Hex Motor
        public static double inchesPerTick = wheelCircumferenceInches / encoderTicksPerRevolution;
        
        // Odometry wheel distances (in cm)
        public static double lateralDistance = 28.0; // Distance between left and right encoders
        public static double forwardOffset = -6.90; // Distance from center to auxiliary encoder
    }
    
    // ===== DRIVE TRAIN PARAMETERS =====
    public static class DriveTrain {
        public static double maxPower = 1.0;
        public static double turnDamping = 0.5;
        public static double motorPowerMultiplier = 1.0;
        
        // Motor direction configuration
        public static boolean reverseFrontLeft = true;
        public static boolean reverseBackLeft = true;
        public static boolean reverseFrontRight = false;
        public static boolean reverseBackRight = false;
        
        // Hardware names
        public static String frontLeftMotorName = "front_left";
        public static String backLeftMotorName = "back_left";
        public static String backRightMotorName = "back_right";
        public static String frontRightMotorName = "front_right";
    }
    
    // ===== PATH PLANNING PARAMETERS =====
    public static class PathPlanning {
        public static double maxPowerStepUp = 0.05;
        public static double powerScaling = 1.0;
        public static double targetZoneInches = 1.0;
        public static double targetZoneDegrees = 10.0;
        public static double rampReachInches = 24.0;
        public static double rampReachDegrees = 45.0;
        public static double minVelocityInchPerSec = 2.0;
        public static double minVelocityDegPerSec = 4.0;
        public static double approachPowerXY = 0.2;
        public static double approachPowerTurn = 0.1;
        public static double breakPower = 0.05;
        public static double breakPowerScale = 0.5;
    }
    
    // ===== TELEOP PARAMETERS =====
    public static class TeleOp {
        public static double maxXYPowerStep = 0.3;
        public static double turnSensitivity = 0.4;
        public static int telemetryUpdateCycles = 10;
    }
    
    // ===== AUTONOMOUS PARAMETERS =====
    public static class Autonomous {
        public static int defaultPathNumber = 1;
        public static int testCycles = 4;
        public static int runTestMs = 100;
        public static double energyThreshold = 8000.0;
    }
    
    // ===== GAME-SPECIFIC PARAMETERS =====
    public static class Game {
        public static double fieldWidthInches = 144.0; // 12 feet
        public static double fieldLengthInches = 144.0; // 12 feet
        public static double roombaRadius = 20.0;
        
        public enum Alliance {
            RED, BLUE
        }
        
        public enum StartingPosition {
            LEFT, CENTER, RIGHT
        }
        
        public static Alliance currentAlliance = Alliance.RED;
        public static StartingPosition startingPosition = StartingPosition.LEFT;
    }
    
    // ===== IMU PARAMETERS =====
    public static class IMU {
        public static String imuName = "imu";
        public static double angleOffset = 0.0;
    }
    
    // ===== CAMERA PARAMETERS =====
    public static class Camera {
        public static String webcamName = "Webcam 1";
        public static int aprilTagDetectionId = 0;
        public static boolean aprilTagDetectionEnabled = false;
    }
    
    // ===== ODOMETRY PARAMETERS =====
    public static class Odometry {
        public enum Type {
            DEADWHEEL, XYPLUSIMU, SIMULATION
        }
        
        public static Type odometryType = Type.SIMULATION;
        
        // Deadwheel odometry parameters
        public static double deadwheelRadius = 2.4; // GoBilda odometry wheel radius in cm (48mm diameter)
        public static int deadwheelTicksPerRevolution = 2000; // GoBilda odometry pod: 2000 Countable Events per Revolution
        
        // XY+IMU odometry parameters
        public static double xyImuWheelRadius = 4.8; // Mecanum wheel radius in cm (OD=96mm)
        public static double xyImuTicksPerRevolution = 537.7; // 312 RPM motor encoder tics per revolution (PPR)
        public static double xyImuStrafeCmPerTick = 73.2/1477; // measured with coach chassis
        
        // Physical odometry wheel distances (in cm) - these are already in Physical class but repeated here for clarity
        public static double lateralDistance = Physical.lateralDistance; // Distance between left and right encoders
        public static double forwardOffset = Physical.forwardOffset; // Distance from center to auxiliary encoder
        
        // Simulation parameters
        public static double simulationForwardRate = 0.035; // inches per ms per power unit
        public static double simulationStrafeRate = 0.035;  // inches per ms per power unit
        public static double simulationTurnRate = 0.4;      // degrees per ms per power unit
    }
}
