package org.firstinspires.ftc.teamcode.op;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configuration.RobotConfig;
import org.firstinspires.ftc.teamcode.pathmaker.PathDetails;
import org.firstinspires.ftc.teamcode.pathmaker.PathMakerStateMachine;
import org.firstinspires.ftc.teamcode.pathmaker.PathManager;

/**
 * Refactored Autonomous OpMode that provides a cleaner, more maintainable
 * structure for autonomous robot control. This class extends BaseOpMode
 * to inherit common functionality while providing autonomous-specific logic.
 */
@Config
@Autonomous(name = "Refactored Auto", group = "Competition")
public class RefactoredAutoOpMode extends BaseOpMode {
    
    // ===== CONFIGURATION PARAMETERS =====
    public static int selectedPathNumber = 1;
    public static boolean enableAprilTagDetection = false;
    public static int aprilTagId = 0;
    public static double testDurationMs = 100;
    
    // ===== PRIVATE FIELDS =====
    private ElapsedTime pathTimer;
    private int telemetryUpdateCycles;
    private int currentPathStep;
    private boolean pathComplete;
    
    // ===== INITIALIZATION =====
    
    @Override
    protected void initializeRobot() {
        // Initialize autonomous-specific systems
        pathTimer = new ElapsedTime();
        telemetryUpdateCycles = 0;
        currentPathStep = 0;
        pathComplete = false;
        
        // Set autonomous mode
        PathMakerStateMachine.setAutonomous();
        
        // Initialize path planning system
        PathDetails.initAutoPathList();
        
        // Configure April Tag detection if enabled
        if (enableAprilTagDetection) {
            PathMakerStateMachine.aprilTagDetectionOn = true;
            PathMakerStateMachine.aprilTagDetectionID = aprilTagId;
        }
        
        // Log initialization
        dashboardTelemetry.addData("Auto Mode", "Initialized");
        dashboardTelemetry.addData("Selected Path", selectedPathNumber);
        dashboardTelemetry.addData("April Tag Detection", enableAprilTagDetection);
    }
    
    // ===== MAIN LOOP =====
    
    @Override
    protected void runLoop() throws InterruptedException {
        // Update robot pose
        RobotPose.readPose();
        
        // Handle different autonomous modes
        if (selectedPathNumber == -1) {
            runTestMode();
        } else if (selectedPathNumber == 0) {
            runDashboardControlledMode();
        } else {
            runPathFollowingMode();
        }
        
        // Update telemetry periodically
        updateTelemetry();
    }
    
    // ===== AUTONOMOUS MODES =====
    
    /**
     * Run test mode for debugging and calibration.
     */
    private void runTestMode() throws InterruptedException {
        // Use dashboard parameters for testing
        double forwardPower = 0.0; // Example
        double strafePower = 0.0;
        double turnPower = 0.2;
        double headingDrive = 0.0;
        
        WheelPowerManager.setDrivePower(driveTrain, forwardPower, strafePower, turnPower, headingDrive);
        sleep((long) testDurationMs);
        WheelPowerManager.setDrivePower(driveTrain, 0, 0, 0, 0);
    }
    
    /**
     * Run dashboard-controlled mode for manual testing.
     */
    private void runDashboardControlledMode() {
        // Reset robot pose to origin
        RobotPose.setPose(0, 0, 0);
        
        // Set up for manual path control
        PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_SET_PATH;
        PathMakerStateMachine.currentPath = PathMakerStateMachine.nextPath = 0;
    }
    
    /**
     * Run the main path following autonomous mode.
     */
    private void runPathFollowingMode() throws InterruptedException {
        // Update path following state machine
        PathMakerStateMachine.updateAuto(dashboardTelemetry);
        
        // Check if path is complete
        if (PathMakerStateMachine.pm_state == PathMakerStateMachine.PM_STATE.DONE) {
            pathComplete = true;
        }
    }
    
    // ===== TELEMETRY =====
    
    /**
     * Update telemetry with autonomous-specific information.
     */
    private void updateTelemetry() {
        telemetryUpdateCycles++;
        
        if (telemetryUpdateCycles > 10) {
            // Update common telemetry
            updateCommonTelemetry();
            
            // Add path goal telemetry
            addPathGoalTelemetry();
            
            // Add autonomous-specific telemetry
            dashboardTelemetry.addData("Path State", PathMakerStateMachine.pm_state);
            dashboardTelemetry.addData("Current Path", getCurrentPathDescription());
            dashboardTelemetry.addData("Path Complete", pathComplete);
            dashboardTelemetry.addData("Path Timer", String.format("%.1f", pathTimer.milliseconds()));
            
            // Add path planning telemetry
            if (selectedPathNumber > 0) {
                addPathPlanningTelemetry();
            }
            
            // Add April Tag telemetry if enabled
            if (enableAprilTagDetection) {
                addAprilTagTelemetry();
            }
            
            dashboardTelemetry.update();
            telemetryUpdateCycles = 0;
        }
    }
    
    /**
     * Add path planning specific telemetry.
     */
    private void addPathPlanningTelemetry() {
        dashboardTelemetry.addLine(String.format("Path Time: %.1f ms", PathDetails.elapsedTime_ms.milliseconds()));
        dashboardTelemetry.addLine(String.format("Delta Y/X/A: %.1f / %.1f / %.1f",
                PathManager.deltaIsShouldY,
                PathManager.deltaIsShouldX,
                PathManager.deltaIsShouldAngle));
        dashboardTelemetry.addLine(String.format("Power Y/X/A: %.2f / %.2f / %.2f",
                PathManager.yPower,
                PathManager.xPower,
                PathManager.turnPower));
        dashboardTelemetry.addLine(String.format("Velocity Y/X/A: %.1f / %.1f / %.1f",
                RobotPose.getYVelocity_inPerSec(),
                RobotPose.getXVelocity_inPerSec(),
                RobotPose.getHeadingVelocity_degPerSec()));
    }
    
    /**
     * Add April Tag detection telemetry.
     */
    private void addAprilTagTelemetry() {
        dashboardTelemetry.addData("April Tag Detection", PathMakerStateMachine.aprilTagDetectionOn);
        dashboardTelemetry.addData("Target Tag ID", PathMakerStateMachine.aprilTagDetectionID);
        dashboardTelemetry.addData("In Target Zone", PathManager.inTargetZone);
    }
    
    /**
     * Add path goal telemetry.
     */
    private void addPathGoalTelemetry() {
        // Update field goals for telemetry display
        PathDetails.updateFieldGoalsForTelemetry();
        
        dashboardTelemetry.addLine("=== PATH GOALS ===");
        dashboardTelemetry.addData("Rotate Deg", PathDetails.getCurrentRotateDeg());
        dashboardTelemetry.addData("Y Field Goal (in)", String.format("%.1f", PathDetails.yFieldGoal_in));
        dashboardTelemetry.addData("X Field Goal (in)", String.format("%.1f", PathDetails.xFieldGoal_in));
        dashboardTelemetry.addData("A Field Goal (deg)", String.format("%.1f", PathDetails.aFieldGoal_deg));
    }
    
    /**
     * Get a description of the current path.
     */
    private String getCurrentPathDescription() {
        if (PathMakerStateMachine.currentPath < 0) {
            return "No Path";
        }
        
        try {
            if (PathDetails.autoPathList.isEmpty()) {
                return "No Paths Available";
            }
            if (PathMakerStateMachine.currentPath >= PathDetails.autoPathList.size()) {
                return "Path Not Found";
            }
            // Return the description of the current path
            // convert PathDetails.autoPathList.get(...currentPath) to string
            return PathDetails.autoPathList.get(PathMakerStateMachine.currentPath).toString();
        } catch (IndexOutOfBoundsException e) {
            return "Invalid Path";
        }
    }
    
    // ===== UTILITY METHODS =====
    
    /**
     * Check if the autonomous routine is complete.
     */
    public boolean isAutonomousComplete() {
        return pathComplete || getRuntime() > 30.0; // 30 second autonomous period
    }
    
    /**
     * Get the starting heading based on alliance and position.
     */
    
    // ===== CLEANUP =====
    
    @Override
    protected void cleanup() {
        // Stop all motors
        WheelPowerManager.setDrivePower(driveTrain, 0, 0, 0, 0);
        
        // Log completion
        dashboardTelemetry.addData("Auto Complete", "Runtime: " + String.format("%.1f", getRuntime()));
        dashboardTelemetry.update();
    }
}
