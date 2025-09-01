package org.firstinspires.ftc.teamcode.configuration;

import com.acmerobotics.dashboard.config.Config;

/**
 * Centralized configuration for robot parameters.
 * This class consolidates all robot-specific settings for easy portability
 * between different robot configurations and game layouts.
 */

@Config
public class RobotConfig {


    // Physical parameters
    @Config
    public static class Physical {
        public static double robotMassKg = 10.0;
        public static double wheelDiameterInches = 4.0;
        public static double wheelCircumferenceInches = wheelDiameterInches * Math.PI;
        public static double encoderTicksPerRevolution = 537.6; // REV HD Hex Motor



        // Accessor methods
        public static double getRobotMassKg() { return robotMassKg; }
        public static double getWheelCircumferenceInches() { return wheelCircumferenceInches; }
        public static double getEncoderTicksPerRevolution() { return encoderTicksPerRevolution; }
    }


    // Drive train parameters
    @Config
    public static class DriveTrain {
        
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



        // Accessor methods
        public static boolean isFrontLeftReversed() { return reverseFrontLeft; }
        public static boolean isBackLeftReversed() { return reverseBackLeft; }
        public static boolean isBackRightReversed() { return reverseBackRight; }
        public static boolean isFrontRightReversed() { return reverseFrontRight; }

        public static String getFrontLeftMotorName() { return frontLeftMotorName; }
        public static String getBackLeftMotorName() { return backLeftMotorName; }
        public static String getBackRightMotorName() { return backRightMotorName; }
        public static String getFrontRightMotorName() { return frontRightMotorName; }
    }


    // IMU parameters
    public static class IMU {
        public static String imuName = "imu";
        public static String getImuName() { return imuName; }
    }


    // Camera parameters
    public static class Camera {
        public static String webcamOneName = "Webcam 1";
        public static String webcamTwoName = "Webcam 2";
        public static String getWebcamOneName() { return webcamOneName; }
        public static String getWebcamTwoName() { return webcamTwoName; }
    }


    // Odometry parameters
    @Config
    public static class Odometry {

        public enum OdometryType {
            DEADWHEEL, XYPLUSIMU, SIMULATION
        }

        // Type of odometry being used
        public static OdometryType odometryType = OdometryType.DEADWHEEL;

        // Odometry wheel distances (in cm)
        public static double lateralDistance = 30.16; // Distance between left and right encoders
        public static double forwardOffset = -13.3; // Distance from center to auxiliary encoder

        public static boolean flipLeftEncoder = false;
        public static boolean flipMiddleEncoder = false;
        public static boolean flipRightEncoder = false;
        public static boolean flipTurnDirection = false;
        public static boolean flipStrafeTurnCorrection = false;
        
        // Deadwheel odometry parameters
        public static double deadwheelRadius = 2.4; // GoBilda odometry wheel radius in cm (48mm diameter)
        public static int deadwheelTicksPerRevolution = 2000; // GoBilda odometry pod: 2000 Countable Events per Revolution
        
        // XY+IMU odometry parameters
        public static double xyImuWheelRadius = 4.8; // Mecanum wheel radius in cm (OD=96mm)
        public static double xyImuTicksPerRevolution = 537.7; // 312 RPM motor encoder tics per revolution (PPR)
        public static double xyImuStrafeCmPerTick = 73.2/1477; // measured with coach chassis

        // Simulation parameters
        public static double simulationForwardRate = 0.035; // inches per ms per power unit
        public static double simulationStrafeRate = 0.035;  // inches per ms per power unit
        public static double simulationTurnRate = 0.4;      // degrees per ms per power unit



        // Accessor methods
        public static OdometryType getOdometryType() { return odometryType; }

        public static double getLateralDistance() { return lateralDistance; }
        public static double getForwardOffset() { return forwardOffset; }

        public static boolean isLeftEncoderFlipped() { return flipLeftEncoder; }
        public static boolean isMiddleEncoderFlipped() { return flipMiddleEncoder; }
        public static boolean isRightEncoderFlipped() { return flipRightEncoder; }
        public static boolean isTurnDirectionFlipped() { return flipTurnDirection; }
        public static boolean isStrafeTurnCorrectionFlipped() { return flipStrafeTurnCorrection; }

        public static double getDeadwheelRadius() { return deadwheelRadius; }
        public static double getDeadwheelTicksPerRevolution() { return deadwheelTicksPerRevolution; }

        public static double getImuWheelRadius() { return xyImuWheelRadius; }
        public static double getImuTicksPerRevolution() { return xyImuTicksPerRevolution; }
        public static double getImuStrafePerTick() { return xyImuStrafeCmPerTick; }

        public static double simulationForwardRate() { return simulationForwardRate; }
        public static double getSimulationStrafeRate() { return simulationStrafeRate; }
        public static double getSimulationTurnRate() { return simulationTurnRate; }
    }
}