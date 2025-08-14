package org.firstinspires.ftc.teamcode.op;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.GameConfig;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.pathmaker.PathDetails;
import org.firstinspires.ftc.teamcode.pathmaker.PathMakerStateMachine;
import org.firstinspires.ftc.teamcode.pathmaker.PathManager;

/**
 * Refactored TeleOp OpMode that provides a cleaner, more maintainable
 * structure for driver-controlled robot operation. This class extends BaseOpMode
 * to inherit common functionality while providing teleop-specific logic.
 */
@Config
@TeleOp(name = "Refactored TeleOp", group = "Competition")
public class RefactoredTeleOpMode extends BaseOpMode {
    
    // ===== CONFIGURATION PARAMETERS =====
    public static double motorPowerMultiplier = 1.0;
    public static double turnDamping = 0.5;
    public static boolean enableFieldCentric = true;
    public static boolean enableAprilTagAlignment = false;
    public static int aprilTagId = 0;
    
    // ===== PRIVATE FIELDS =====
    private ElapsedTime telemetryTimer;
    private int telemetryUpdateCycles;
    private double lastYPower, lastXPower, lastTurnPower;
    private boolean aprilTagAlignmentActive;
    
    // ===== INITIALIZATION =====
    
    @Override
    protected void initializeRobot() {
        // Initialize teleop-specific systems
        telemetryTimer = new ElapsedTime();
        telemetryUpdateCycles = 0;
        lastYPower = 0.0;
        lastXPower = 0.0;
        lastTurnPower = 0.0;
        aprilTagAlignmentActive = false;
        
        // Set driver-controlled mode
        PathMakerStateMachine.setDriverControlled();
        
        // Initialize path planning system
        PathDetails.initializePath();
        
        // Configure April Tag alignment if enabled
        if (enableAprilTagAlignment) {
            PathMakerStateMachine.aprilTagDetectionOn = true;
            PathMakerStateMachine.aprilTagDetectionID = aprilTagId;
        }
        
        // Log initialization
        dashboardTelemetry.addData("TeleOp Mode", "Initialized");
        dashboardTelemetry.addData("Field Centric", enableFieldCentric);
        dashboardTelemetry.addData("April Tag Alignment", enableAprilTagAlignment);
        dashboardTelemetry.addData("Motor Power Multiplier", motorPowerMultiplier);
        dashboardTelemetry.update();
    }
    
    // ===== MAIN LOOP =====
    
    @Override
    protected void runLoop() throws InterruptedException {
        // Update robot pose
        RobotPose.readPose();
        
        // Handle driver input
        handleDriverInput();
        
        // Update simulation with motor powers if in simulation mode
        RobotPose.updateSimulationWithMotorPowers(lastYPower, lastXPower, lastTurnPower);
        
        // Handle April Tag alignment if enabled
        if (enableAprilTagAlignment) {
            handleAprilTagAlignment();
        }
        
        // Update path manager
        PathMakerStateMachine.updateTele(gamepad1, dashboardTelemetry);
        
        // Apply motor powers
        applyMotorPowers();
        
        // Update telemetry periodically
        updateTelemetry();
    }
    
    // ===== DRIVER INPUT HANDLING =====
    
    /**
     * Process driver input from gamepad and convert to robot movement.
     */
    private void handleDriverInput() {
        // Get raw gamepad values
        double rawY = -gamepad1.left_stick_y; // Forward/backward (inverted for intuitive control)
        double rawX = gamepad1.left_stick_x;  // Strafe left/right
        double rawTurn = gamepad1.right_stick_x; // Turn left/right
        
        // Apply deadzone to prevent drift
        rawY = applyDeadzone(rawY, 0.1);
        rawX = applyDeadzone(rawX, 0.1);
        rawTurn = applyDeadzone(rawTurn, 0.1);
        
        // Apply power scaling and damping
        double yPower = rawY * motorPowerMultiplier;
        double xPower = rawX * motorPowerMultiplier;
        double turnPower = rawTurn * motorPowerMultiplier * turnDamping;
        
        // Store for telemetry
        lastYPower = yPower;
        lastXPower = xPower;
        lastTurnPower = turnPower;
        
        // Set powers in state machine
        PathMakerStateMachine.yPower = yPower;
        PathMakerStateMachine.xPower = xPower;
        PathMakerStateMachine.turnPower = turnPower;
    }
    
    /**
     * Apply deadzone to prevent controller drift.
     */
    private double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0.0;
        }
        return value;
    }
    
    /**
     * Handle April Tag alignment for precise positioning.
     */
    private void handleAprilTagAlignment() {
        // Check if April Tag alignment is requested (e.g., left bumper)
        if (gamepad1.left_bumper && !aprilTagAlignmentActive) {
            // Start April Tag alignment
            aprilTagAlignmentActive = true;
            PathMakerStateMachine.aprilTagDetectionOn = true;
            
            // Set up alignment path
            PathDetails.autoAprilTagAndFieldGoals();
            PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_APRILTAG_ExecutePath;
            PathDetails.elapsedTime_ms.reset();
            
            dashboardTelemetry.addData("April Tag Alignment", "Started");
        }
        
        // Check if alignment should be cancelled
        if (gamepad1.right_bumper && aprilTagAlignmentActive) {
            aprilTagAlignmentActive = false;
            PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.DRIVER_CONTROL;
            dashboardTelemetry.addData("April Tag Alignment", "Cancelled");
        }
    }
    
    /**
     * Apply motor powers to the drive train.
     */
    private void applyMotorPowers() {
        // Get powers from path manager (may be modified by April Tag alignment)
        double yPower = PathManager.yPower;
        double xPower = PathManager.xPower;
        double turnPower = PathManager.turnPower;
        
        // Apply field-centric transformation if enabled
        if (enableFieldCentric && !aprilTagAlignmentActive) {
            double[] fieldCentricPowers = convertToFieldCentric(yPower, xPower, turnPower);
            yPower = fieldCentricPowers[0];
            xPower = fieldCentricPowers[1];
            turnPower = fieldCentricPowers[2];
        }
        
        // Apply powers to drive train
        WheelPowerManager.setDrivePower(driveTrain, yPower, xPower, turnPower, 0.0);
    }
    
    /**
     * Convert robot-centric powers to field-centric powers.
     */
    private double[] convertToFieldCentric(double yPower, double xPower, double turnPower) {
        double robotHeading = RobotPose.getFieldAngle_deg();
        double headingRadians = Math.toRadians(robotHeading);
        
        // Rotate the power vectors by the robot's heading
        double fieldYPower = yPower * Math.cos(headingRadians) - xPower * Math.sin(headingRadians);
        double fieldXPower = yPower * Math.sin(headingRadians) + xPower * Math.cos(headingRadians);
        
        return new double[]{fieldYPower, fieldXPower, turnPower};
    }
    
    // ===== TELEMETRY =====
    
    /**
     * Update telemetry with teleop-specific information.
     */
    private void updateTelemetry() {
        telemetryUpdateCycles++;
        
        if (telemetryUpdateCycles > RobotConfig.TeleOp.telemetryUpdateCycles) {
            // Update common telemetry
            updateCommonTelemetry();
            
            // Add teleop-specific telemetry
            addDriverInputTelemetry();
            addMotorPowerTelemetry();
            addPathGoalTelemetry();
            addAprilTagTelemetry();
            addPerformanceTelemetry();
            
            dashboardTelemetry.update();
            telemetryUpdateCycles = 0;
        }
    }
    
    /**
     * Add driver input telemetry.
     */
    private void addDriverInputTelemetry() {
        dashboardTelemetry.addLine("=== DRIVER INPUT ===");
        dashboardTelemetry.addLine(String.format("Gamepad Y/X/A: %.2f / %.2f / %.2f",
                -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x));
        dashboardTelemetry.addLine(String.format("Raw Y/X/A: %.2f / %.2f / %.2f",
                lastYPower, lastXPower, lastTurnPower));
        dashboardTelemetry.addData("Field Centric", enableFieldCentric);
        dashboardTelemetry.addData("Motor Power Multiplier", motorPowerMultiplier);
    }
    
    /**
     * Add motor power telemetry.
     */
    private void addMotorPowerTelemetry() {
        dashboardTelemetry.addLine("=== MOTOR POWERS ===");
        dashboardTelemetry.addLine(String.format("Applied Y/X/A: %.2f / %.2f / %.2f",
                PathManager.yPower, PathManager.xPower, PathManager.turnPower));
        dashboardTelemetry.addLine(String.format("Last Y/X/A: %.2f / %.2f / %.2f",
                PathManager.yPowerLast, PathManager.xPowerLast, PathManager.turnPowerLast));
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
     * Add April Tag telemetry.
     */
    private void addAprilTagTelemetry() {
        if (enableAprilTagAlignment) {
            dashboardTelemetry.addLine("=== APRIL TAG ===");
            dashboardTelemetry.addData("Alignment Active", aprilTagAlignmentActive);
            dashboardTelemetry.addData("Tag Detection", PathMakerStateMachine.aprilTagDetectionOn);
            dashboardTelemetry.addData("Target Tag ID", PathMakerStateMachine.aprilTagDetectionID);
            dashboardTelemetry.addData("In Target Zone", PathManager.inTargetZone);
            
            if (aprilTagAlignmentActive) {
                double[] tagOffset = RobotPose.tagOffset(PathMakerStateMachine.aprilTagDetectionID);
                dashboardTelemetry.addLine(String.format("Tag Offset X/Y/A: %.1f / %.1f / %.1f",
                        tagOffset[0], tagOffset[1], tagOffset[2]));
            }
        }
    }
    
    /**
     * Add performance telemetry.
     */
    private void addPerformanceTelemetry() {
        dashboardTelemetry.addLine("=== PERFORMANCE ===");
        dashboardTelemetry.addLine(String.format("Velocity Y/X/A: %.1f / %.1f / %.1f",
                RobotPose.getYVelocity_inPerSec(),
                RobotPose.getXVelocity_inPerSec(),
                RobotPose.getHeadingVelocity_degPerSec()));
        dashboardTelemetry.addLine(String.format("Loop Time: %.1f ms", telemetryTimer.milliseconds()));
    }
    
    // ===== UTILITY METHODS =====
    
    /**
     * Get the starting heading for teleop (usually 0 for field-centric control).
     */
    @Override
    protected double getStartingHeading() {
        // For teleop, we typically start at 0 degrees for field-centric control
        return 0.0;
    }
    
    /**
     * Check if the robot is in a safe state for teleop.
     */
    @Override
    protected boolean isRobotSafe() {
        // Add teleop-specific safety checks
        // For example, check if robot is within field boundaries
        double x = RobotPose.getFieldX_in();
        double y = RobotPose.getFieldY_in();
        
        boolean withinBounds = Math.abs(x) <= GameConfig.Field.halfWidth && 
                              Math.abs(y) <= GameConfig.Field.halfLength;
        
        if (!withinBounds) {
            dashboardTelemetry.addData("SAFETY WARNING", "Robot outside field bounds!");
        }
        
        return withinBounds;
    }
    
    // ===== CLEANUP =====
    
    @Override
    protected void cleanup() {
        // Stop all motors
        WheelPowerManager.setDrivePower(driveTrain, 0, 0, 0, 0);
        
        // Reset April Tag detection
        PathMakerStateMachine.aprilTagDetectionOn = false;
        
        // Log completion
        dashboardTelemetry.addData("TeleOp Complete", "Runtime: " + String.format("%.1f", getRuntime()));
        dashboardTelemetry.update();
    }
}
