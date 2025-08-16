package org.firstinspires.ftc.teamcode.op;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configuration.RobotConfig;
import org.firstinspires.ftc.teamcode.configuration.GameConfig;
import org.firstinspires.ftc.teamcode.hw.DriveTrain;
import org.firstinspires.ftc.teamcode.hw.MyIMU;
import org.firstinspires.ftc.teamcode.pathmaker.PathMakerStateMachine;

/**
 * Base OpMode class that provides common functionality for both
 * autonomous and teleop modes. This reduces code duplication and
 * provides a consistent initialization process.
 */
public abstract class BaseOpMode extends LinearOpMode {
    
    // ===== PROTECTED FIELDS =====
    protected DriveTrain driveTrain;
    protected MyIMU imu;
    protected RobotPose robotPose;
    protected Telemetry dashboardTelemetry;
    protected ElapsedTime runtime = new ElapsedTime(); // Initialize immediately to prevent null pointer
    
    // ===== ABSTRACT METHODS =====
    /**
     * Initialize robot-specific hardware and systems.
     * Called once during initialization.
     */
    protected abstract void initializeRobot();
    
    /**
     * Main loop logic for the OpMode.
     * Called repeatedly while the OpMode is active.
     */
    protected abstract void runLoop() throws InterruptedException;
    
    /**
     * Cleanup resources when the OpMode stops.
     * Called once when the OpMode is stopped.
     */
    protected abstract void cleanup();
    
    // ===== INITIALIZATION METHODS =====
    
    /**
     * Initialize common systems used by all OpModes.
     */
    protected void initializeCommon() throws InterruptedException {
        // Runtime timer is already initialized in field declaration
        
        // Initialize dashboard telemetry
        dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        
        // Initialize drive train
        driveTrain = new DriveTrain(this);
        driveTrain.init();
        
        // Initialize IMU
        imu = new MyIMU(this);
        MyIMU.init(this);
        MyIMU.resetAngle();
        
        // Initialize robot pose tracking
        robotPose = new RobotPose();
        RobotPose.initializePose(this, driveTrain, dashboardTelemetry);
        
        // Set initial pose based on game configuration
        setInitialPose();
        
        // Initialize robot-specific systems
        initializeRobot();
        
        // Update telemetry
        updateInitialTelemetry();
    }
    
    /**
     * Set the initial robot pose based on game configuration.
     */
    protected void setInitialPose() {
        double startX = GameConfig.StartingPositions.getStartingX(
            RobotConfig.Game.currentAlliance, 
            RobotConfig.Game.startingPosition
        );
        double startY = GameConfig.StartingPositions.getStartingY(
            RobotConfig.Game.currentAlliance, 
            RobotConfig.Game.startingPosition
        );
        double startHeading = getStartingHeading();
        
        RobotPose.setPose(startX, startY, startHeading);
    }
    
    /**
     * Get the starting heading based on alliance and position.
     */
    protected double getStartingHeading() {
        // Default heading - can be overridden by subclasses
        return 0.0;
    }
    
    /**
     * Update telemetry with initial robot state.
     */
    protected void updateInitialTelemetry() {
        dashboardTelemetry.addData("Alliance", RobotConfig.Game.currentAlliance);
        dashboardTelemetry.addData("Starting Position", RobotConfig.Game.startingPosition);
        dashboardTelemetry.addData("Initial X", RobotPose.getFieldX_in());
        dashboardTelemetry.addData("Initial Y", RobotPose.getFieldY_in());
        dashboardTelemetry.addData("Initial Heading", RobotPose.getFieldAngle_deg());
        dashboardTelemetry.update();
    }
    
    // ===== UTILITY METHODS =====
    
    /**
     * Check if the robot is in a safe state to continue operation.
     */
    protected boolean isRobotSafe() {
        // Add safety checks here (e.g., motor temperatures, battery voltage)
        return true;
    }
    
    /**
     * Get the current runtime in seconds.
     */
    public double getRuntime() {
        return runtime.seconds();
    }
    
    /**
     * Get the current runtime in milliseconds.
     */
    protected double getRuntimeMs() {
        return runtime.milliseconds();
    }
    
    /**
     * Update telemetry with common robot information.
     */
    protected void updateCommonTelemetry() {
        dashboardTelemetry.addData("Runtime", String.format("%.1f", getRuntime()));
        dashboardTelemetry.addData("Robot X", String.format("%.1f", RobotPose.getFieldX_in()));
        dashboardTelemetry.addData("Robot Y", String.format("%.1f", RobotPose.getFieldY_in()));
        dashboardTelemetry.addData("Robot Heading", String.format("%.1f", RobotPose.getFieldAngle_deg()));
        dashboardTelemetry.addData("Robot Safe", isRobotSafe());
    }
    
    // ===== MAIN OP_MODE METHODS =====
    
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            // Initialize common systems
            initializeCommon();
            
            // Wait for start
            waitForStart();
            runtime.reset();
            
            // Main loop
            while (opModeIsActive()) {
                if (isStopRequested()) {
                    break;
                }
                
                // Check robot safety
                if (!isRobotSafe()) {
                    telemetry.addData("WARNING", "Robot not in safe state!");
                    telemetry.update();
                    // Implement safety shutdown if needed
                }
                
                // Run the main loop logic
                runLoop();
                
                // Small delay to prevent overwhelming the system
                sleep(10);
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Exception in OpMode: " + e.getMessage());
            telemetry.update();
        } finally {
            // Cleanup
            cleanup();
        }
    }
}
