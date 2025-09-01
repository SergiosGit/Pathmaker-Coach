// RobotPoseSimulation.java
//
// This class provides simulated robot pose tracking for testing and debugging purposes.
// It uses simple physics calculations based on motor powers to simulate robot movement.
//
// MIT License
// Copyright (c) 2023 bayrobotics.org
//
package org.firstinspires.ftc.teamcode.op;

import org.firstinspires.ftc.teamcode.configuration.RobotConfig;
import org.firstinspires.ftc.teamcode.configuration.GameConfig;

/**
 * Simulated robot pose tracking for testing and debugging.
 * This class provides a physics-based simulation of robot movement
 * that can be used when real odometry hardware is not available.
 */
public class RobotPoseSimulation {
    
    // ===== SIMULATION STATE =====
    private static double simulatedForward_in = 0.0;
    private static double simulatedStrafe_in = 0.0;
    private static double simulatedAngle_deg = 0.0;
    private static double simulatedFieldX_in = 0.0;
    private static double simulatedFieldY_in = 0.0;
    private static double simulatedFieldAngle_deg = 0.0;
    
    // Previous values for velocity calculations
    private static double lastSimulatedForward_in = 0.0;
    private static double lastSimulatedStrafe_in = 0.0;
    private static double lastSimulatedAngle_deg = 0.0;
    private static double lastSimulatedFieldX_in = 0.0;
    private static double lastSimulatedFieldY_in = 0.0;
    
    // ===== SIMULATION PARAMETERS =====
    private static double forwardPowerToInchRate = 0.035; // inches per ms per power unit
    private static double strafePowerToInchRate = 0.035;  // inches per ms per power unit
    private static double turnPowerToDegRate = 0.4;       // degrees per ms per power unit
    
    // Robot dimensions for visualization
    private static final double ROBOT_WIDTH_IN = 12.0;   // Robot width in inches
    private static final double ROBOT_LENGTH_IN = 16.0;  // Robot length in inches
    
    // ===== VISUALIZATION POINTS =====
    public static double[] robotCornerX = new double[4];
    public static double[] robotCornerY = new double[4];
    
    /**
     * Initialize the simulation with starting position and heading.
     */
    public static void initializeSimulation(double startX_in, double startY_in, double startAngle_deg) {
        simulatedForward_in = startY_in;
        simulatedStrafe_in = startX_in;
        simulatedAngle_deg = startAngle_deg;
        
        // Initialize field coordinates
        updateFieldCoordinates();
        
        // Initialize previous values
        lastSimulatedForward_in = simulatedForward_in;
        lastSimulatedStrafe_in = simulatedStrafe_in;
        lastSimulatedAngle_deg = simulatedAngle_deg;
        lastSimulatedFieldX_in = simulatedFieldX_in;
        lastSimulatedFieldY_in = simulatedFieldY_in;
        
        // Update visualization points
        updateRobotVisualization();
    }
    
    /**
     * Update simulated pose based on motor powers and time step.
     */
    public static void updateSimulatedPose(double forwardPower, double strafePower, double turnPower, double timeStep_ms) {
        // Store previous values for velocity calculations
        lastSimulatedForward_in = simulatedForward_in;
        lastSimulatedStrafe_in = simulatedStrafe_in;
        lastSimulatedAngle_deg = simulatedAngle_deg;
        lastSimulatedFieldX_in = simulatedFieldX_in;
        lastSimulatedFieldY_in = simulatedFieldY_in;
        
        // Apply simple physics simulation
        // Forward movement (robot-centric)
        simulatedForward_in += forwardPower * forwardPowerToInchRate * timeStep_ms;
        
        // Strafe movement (robot-centric)
        simulatedStrafe_in += strafePower * strafePowerToInchRate * timeStep_ms;
        
        // Rotation (robot-centric)
        simulatedAngle_deg += turnPower * turnPowerToDegRate * timeStep_ms;
        
        // Normalize angle to -180 to +180 degrees
        simulatedAngle_deg = normalizeAngle(simulatedAngle_deg);
        
        // Update field coordinates
        updateFieldCoordinates();
        
        // Update visualization
        updateRobotVisualization();
    }
    
    /**
     * Update field coordinates based on robot-centric coordinates and team configuration.
     */
    private static void updateFieldCoordinates() {
        // Convert robot-centric coordinates to field coordinates
        // This accounts for team color and starting position
        
        double fieldOffsetX = 0.0;
        double fieldOffsetY = 0.0;
        
        // Convert robot-centric to field-centric coordinates
        double angleRad = Math.toRadians(simulatedAngle_deg);
        double cos = Math.cos(angleRad);
        double sin = Math.sin(angleRad);
        
        // Transform coordinates
        simulatedFieldX_in = fieldOffsetX + simulatedStrafe_in * cos - simulatedForward_in * sin;
        simulatedFieldY_in = fieldOffsetY + simulatedStrafe_in * sin + simulatedForward_in * cos;
        simulatedFieldAngle_deg = simulatedAngle_deg;
    }
    
    /**
     * Update robot visualization points for dashboard display.
     */
    private static void updateRobotVisualization() {
        double halfWidth = ROBOT_WIDTH_IN / 2.0;
        double halfLength = ROBOT_LENGTH_IN / 2.0;
        
        // Define robot corners in robot-centric coordinates
        double[] robotX = {halfLength, -halfLength, -halfLength, halfLength};
        double[] robotY = {halfWidth, halfWidth, -halfWidth, -halfWidth};
        
        // Rotate and translate to field coordinates
        double angleRad = Math.toRadians(simulatedFieldAngle_deg);
        double cos = Math.cos(angleRad);
        double sin = Math.sin(angleRad);
        
        for (int i = 0; i < 4; i++) {
            // Rotate
            double rotatedX = robotX[i] * cos - robotY[i] * sin;
            double rotatedY = robotX[i] * sin + robotY[i] * cos;
            
            // Translate to field position
            robotCornerX[i] = simulatedFieldX_in + rotatedX;
            robotCornerY[i] = simulatedFieldY_in + rotatedY;
        }
    }
    
    /**
     * Normalize angle to -180 to +180 degrees.
     */
    private static double normalizeAngle(double angle) {
        while (angle > 180.0) {
            angle -= 360.0;
        }
        while (angle < -180.0) {
            angle += 360.0;
        }
        return angle;
    }
    
    // ===== GETTER METHODS =====
    
    /**
     * Get simulated forward position (robot-centric).
     */
    public static double getSimulatedForward_in() {
        return simulatedForward_in;
    }
    
    /**
     * Get simulated strafe position (robot-centric).
     */
    public static double getSimulatedStrafe_in() {
        return simulatedStrafe_in;
    }
    
    /**
     * Get simulated angle (robot-centric).
     */
    public static double getSimulatedAngle_deg() {
        return simulatedAngle_deg;
    }
    
    /**
     * Get simulated field X position.
     */
    public static double getSimulatedFieldX_in() {
        return simulatedFieldX_in;
    }
    
    /**
     * Get simulated field Y position.
     */
    public static double getSimulatedFieldY_in() {
        return simulatedFieldY_in;
    }
    
    /**
     * Get simulated field angle.
     */
    public static double getSimulatedFieldAngle_deg() {
        return simulatedFieldAngle_deg;
    }
    
    // ===== VELOCITY CALCULATIONS =====
    
    /**
     * Get simulated forward velocity (inches per second).
     */
    public static double getSimulatedForwardVelocity_inPerSec(double timeStep_ms) {
        double timeStep_sec = timeStep_ms / 1000.0;
        return (simulatedForward_in - lastSimulatedForward_in) / timeStep_sec;
    }
    
    /**
     * Get simulated strafe velocity (inches per second).
     */
    public static double getSimulatedStrafeVelocity_inPerSec(double timeStep_ms) {
        double timeStep_sec = timeStep_ms / 1000.0;
        return (simulatedStrafe_in - lastSimulatedStrafe_in) / timeStep_sec;
    }
    
    /**
     * Get simulated angular velocity (degrees per second).
     */
    public static double getSimulatedAngularVelocity_degPerSec(double timeStep_ms) {
        double timeStep_sec = timeStep_ms / 1000.0;
        return (simulatedAngle_deg - lastSimulatedAngle_deg) / timeStep_sec;
    }
    
    /**
     * Get simulated field X velocity (inches per second).
     */
    public static double getSimulatedFieldXVelocity_inPerSec(double timeStep_ms) {
        double timeStep_sec = timeStep_ms / 1000.0;
        return (simulatedFieldX_in - lastSimulatedFieldX_in) / timeStep_sec;
    }
    
    /**
     * Get simulated field Y velocity (inches per second).
     */
    public static double getSimulatedFieldYVelocity_inPerSec(double timeStep_ms) {
        double timeStep_sec = timeStep_ms / 1000.0;
        return (simulatedFieldY_in - lastSimulatedFieldY_in) / timeStep_sec;
    }
    
    // ===== CONFIGURATION METHODS =====
    
    /**
     * Set simulation parameters for more realistic behavior.
     */
    public static void setSimulationParameters(double forwardRate, double strafeRate, double turnRate) {
        forwardPowerToInchRate = forwardRate;
        strafePowerToInchRate = strafeRate;
        turnPowerToDegRate = turnRate;
    }
    
    /**
     * Reset simulation to origin.
     */
    public static void resetSimulation() {
        initializeSimulation(0.0, 0.0, 0.0);
    }
}
