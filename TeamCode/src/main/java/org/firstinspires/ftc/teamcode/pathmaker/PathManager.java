//
// PathManager.java
//
// This method calculates the motor powers to drive the robot along the path. The use of
// battery power is maximized for each DOF. For example, if the path only consists of
// forward motion, the absolute power applied will be 1. However, if forward, strafe, and turn
// are all applied, the absolute power for each DOF will be 1/3.
// Power transitions (acceleration) are ramped with a maximum change of maxPowerStep each timeStep_ms.
// When the robot is within "reach" of the goal, the power is ramped down to 0. The reach
// in each DOF is defined by forwardRampReach_in, strafeRampReach_in, and turnRampReach_deg,
// respectively. The robot will stop when it is within the target zone defined by
// forwardTargetZone_in, strafeTargetZone_in, and turnTargetZone_deg.
//
// MIT License
// Copyright (c) 2023 bayrobotics.org
//

package org.firstinspires.ftc.teamcode.pathmaker;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.op.RobotPose;

@Config
public class PathManager {
    // The PathManager manages the power delivered to the robot drive train
    // It operates in field centric mode and uses the RobotPose to determine the robot position
    // to follow the path defined in PathDetails.
    //
    // The X and Y degrees of freedom in field centric coordinates are mapped to the strafe and forward
    // directions of the robot. The turn degree of freedom is the same as the robot heading and used
    // to determine the mapping (X,Y) to (strafe, forward).
    //
    // The PathManager is called from the PathMakerStateMachine.
    // The PathManager is a blocking call with a fixed time interval defined by timeStep_ms.
    // The PathManager will set the "inTargetZone" flag when the robot is within the target zone.
    // The PathMakerStateMachine will terminate the path when the "inTargetZone" flag is set.
    //
    public static double testRampPower = 0;
    public static int testRampZeros = 2;
    private static int testRampCounter = 0;
    public static double maxPowerStepUp = 0.005; // this is an addition, balancing power is done later
    public static double breakPower = 0.05, breakPowerScale = 0.5, approachPowerXY = 0.2, approachPowerTurn = 0.01;
    public static boolean autonomous_x, autonomous_y, autonomous_a;
    private static double powerThreshold = 0.1, approachPower;
    public static long timeStep_ms = 40;
    public static long PMcycleTime_ms = 0;
    public static enum RAMPTYPE {LINEAR, STEP};
    public static RAMPTYPE rampType, rampType_x, rampType_y, rampType_a;
    public static double yRampReach_in = 24;
    public static double xRampReach_in = 12;
    public static double turnRampReach_deg = 45;
    public static double waitBeforeRamp_ms = 300;
    public static double xTargetZone_in = 1, yTargetZone_in = 1, turnTargetZone_deg = 1;
    public static double xMinVelocity_InchPerSec = 5, yMinVelocity_InchPerSec = 5, turnMinVelocity_degPerSec = 5, v_ramp = 1;
    public static double minPower, xMinPower = 0.2, yMinPower = 0.2, turnMinPower = 0.2;
    public static double powerScalingXY = 1, powerScalingTurn = 1, powerScaling;
    public static double yPower, yPowerLast;
    public static double xPower, xPowerLast;
    public static double turnPower, turnPowerLast;
    private enum THISDOF {Y, X, TURN} // Degrees of freedom
    public static double deltaIsShouldY=1e99, deltaIsShouldX=1e99, deltaIsShouldAngle=1e99;
    public static double deltaVelocity=1.0, deltaPosition=1.0;
    public static double lastXVelocity=0.0, lastYVelocity=0.0, lastTurnVelocity=0.0;
    public static double lastXPosition=0.0, lastYPosition=0.0, lastTurnPosition=0.0;
    public static double deltaXPosition=0.0, deltaYPosition=0.0, deltaTurnPosition=0.0;
    public static boolean inTargetZone = false;
    private static ElapsedTime timer = new ElapsedTime();

    public static void moveRobot() throws InterruptedException {
        powerScalingXY = PathDetails.powerScaling;
        double pathElapsedTime = PathDetails.elapsedTime_ms.milliseconds();
        timer.reset();
        //RobotPose.readPose(); // this is done in the PathMakerStateMachine
        // calculate the power for each degree of freedom in field coordinates
        // In autonomous mode the power is proportional to the distance to the goal.
        // In driver control mode the power is proportional to the gamepad input.
        if (autonomous_x) {
            if (pathElapsedTime >= PathDetails.xFieldDelay_ms) {
                deltaIsShouldX = PathDetails.xFieldGoal_in - RobotPose.getFieldX_in();
                xPower = calculateCorrectionPower(THISDOF.X);
            }
        } else {
            xPower = PathMakerStateMachine.xPower;
        }
        if (autonomous_y) {
            // calculate distance to goal for each DOF, followed by correction power
            // which is proportional to distance to goal but limited by maxPowerStep
            if (pathElapsedTime >= PathDetails.yFieldDelay_ms) {
                deltaIsShouldY = PathDetails.yFieldGoal_in - RobotPose.getFieldY_in();
                yPower = calculateCorrectionPower(THISDOF.Y);
            }
        } else {
            yPower = PathMakerStateMachine.yPower;
        }
        if (autonomous_a) {
            if (pathElapsedTime >= PathDetails.turnFieldDelay_ms) {
                deltaIsShouldAngle = PathDetails.aFieldGoal_deg - RobotPose.getFieldAngle_deg();
                turnPower = calculateCorrectionPower(THISDOF.TURN);
            }
        } else {
            turnPower = PathMakerStateMachine.turnPower;
        }
        if (PathMakerStateMachine.control_mode == PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS) {
            // check if robot is in target zone after calculating isShould above
            if (checkInTargetZone()) return;
        }
        balancePower(); // balance power so it doesn't exceed 1
        RobotPose.updatePose(yPower, xPower, turnPower);  // move robot
        // sleep the time needed to match timeStep_ms
        PMcycleTime_ms = (long) timer.milliseconds() + 1; // 1 ms overhead
        if (PMcycleTime_ms<timeStep_ms) {
            // make sure to wait at least timeStep_ms before next iteration
            // timeStep could be exceeded when sensor data is read too slow (readPose)
            Thread.sleep(timeStep_ms-PMcycleTime_ms);
        }
    }
    private static boolean checkInTargetZone() {
        // latch until inTargetZone is reset in setPath
        if (inTargetZone == true) return true;
        // check if robot is in target zone
        if (Math.abs(deltaIsShouldY) < yTargetZone_in &&
                Math.abs(deltaIsShouldX) < xTargetZone_in &&
                Math.abs(deltaIsShouldAngle) < turnTargetZone_deg) {
            inTargetZone = true;
        } else
            inTargetZone = false;
        return inTargetZone;
    }
    private static void balancePower() {
        // balance power so it doesn't exceed 1
        // remember max |power| is 1 but will be distributed evenly on the 3 DOFs
        double sumPower = Math.abs(yPower) + Math.abs(xPower) + Math.abs(turnPower);
        if (sumPower != 0 & sumPower > 1) {
            yPower /= sumPower;
            xPower /= sumPower;
            turnPower /= sumPower;
        }
        // power scaling only for forward and strafe
        if (powerScalingXY <1) {
            yPower *= powerScalingXY;
            xPower *= powerScalingXY;
        }
    }

    private static double calculateCorrectionPower(THISDOF dof) {
        double rampReach;
        double deltaIsShould;
        double power;
        double signumIsShould;
        double lastPower;
        double initialPowerSignum;
        double thisVelocity, minVelocity;
        if (dof == THISDOF.Y) {
            powerScaling = powerScalingXY;
            deltaIsShould = deltaIsShouldY;
            signumIsShould = Math.signum(deltaIsShouldY);
            rampReach = yRampReach_in;
            lastPower = yPowerLast;
            deltaYPosition = RobotPose.getFieldY_in() - lastYPosition;
            lastYPosition = RobotPose.getFieldY_in();
            thisVelocity = RobotPose.getYVelocity_inPerSec();
            deltaVelocity = thisVelocity - lastYVelocity;
            lastYVelocity = thisVelocity;
            minVelocity = yMinVelocity_InchPerSec;
            initialPowerSignum = PathDetails.yInitialPowerSignum;
            approachPower = approachPowerXY;
            rampType = rampType_y;
            minPower = yMinPower;
        } else if (dof == THISDOF.X) {
            powerScaling = powerScalingXY;
            deltaIsShould = deltaIsShouldX;
            signumIsShould = Math.signum(deltaIsShouldX);
            rampReach = xRampReach_in;
            lastPower = xPowerLast;
            deltaXPosition = RobotPose.getFieldX_in() - lastXPosition;
            thisVelocity = RobotPose.getXVelocity_inPerSec();
            deltaVelocity = thisVelocity - lastXVelocity;
            lastXVelocity = thisVelocity;
            minVelocity = xMinVelocity_InchPerSec;
            initialPowerSignum = PathDetails.xInitialPowerSignum;
            approachPower = approachPowerXY;
            rampType = rampType_x;
            minPower = xMinPower;
        } else {
            powerScaling = powerScalingTurn;
            deltaIsShould = deltaIsShouldAngle;
            signumIsShould = Math.signum(deltaIsShouldAngle);
            rampReach = turnRampReach_deg;
            lastPower = turnPowerLast;
            deltaTurnPosition = RobotPose.getFieldAngle_deg() - lastTurnPosition;
            thisVelocity = RobotPose.getHeadingVelocity_degPerSec();
            deltaVelocity = thisVelocity - lastTurnVelocity;
            lastTurnVelocity = thisVelocity;
            minVelocity = turnMinVelocity_degPerSec;
            initialPowerSignum = PathDetails.aInitialPowerSignum;
            approachPower = approachPowerTurn;
            rampType = rampType_a;
            minPower = turnMinPower;
        }
        // calculate ramp power
        // typical case: rampReach > 0, i.e. robot is supposed to stop at the goal
        // rampReach is negative to indicate that the robot should keep going full power
        // until deltaIsShould reverses (i.e. goal is reached) and then go to the next path
        if (rampReach > 0) { // rampReach is positive
            // outside reach value: move with maximum available power
            if (Math.abs(deltaIsShould) > rampReach) {
                // outside reach value: move with maximum available power
                // for rampReach == 0, the robot will move with maximum power
                // this is an expert mode, the user needs to make sure the robot
                // can stop in time
                power = signumIsShould;
            } else {
                if (rampType == RAMPTYPE.LINEAR) {
                    // within reach value: reduce power proportional to distance to goal
                    // power = deltaIsShould / rampReach;
                    testRampCounter++;
                    // if testRampCounter is equal testRampZeros, use ramp, else set to testRampPower
                    // full breaking if energy in any DOF is above threshold
                    double robot_energy_threshold = RobotPose.testrobot_energy_threshold;
                    boolean isEnergyAboveThreshold = RobotPose.getAngularEnergy() > robot_energy_threshold ||
                            RobotPose.getForwardEnergy() > robot_energy_threshold || RobotPose.getStrafeEnergy() > robot_energy_threshold;
                    if (isEnergyAboveThreshold) {
                        power = 0;
                    } else {
                        power = deltaIsShould / rampReach;
                    }
                } else {
                    double minPower = 0.2 / powerScaling; // undo scaling for minPower
//                    power = signumIsShould * Math.max(minPower,Math.abs(deltaIsShould / rampReach));
//                    // for the first waitBeforeRamp_ms, use linear ramp
//                    if (PathDetails.elapsedTime_ms.milliseconds() > waitBeforeRamp_ms) {
//                        double v_abs = Math.max(Math.abs(thisVelocity), minVelocity/2);
//                        v_ramp = minVelocity / v_abs;
//                        v_ramp = Math.max(v_ramp, 0.9);
//                        v_ramp = Math.min(v_ramp, 1.1);
//                        power *= v_ramp;
//                    }
                    // rate at which velocity needs to be ramped down to zero
                    double v_rate_required = thisVelocity / deltaIsShould;
                    // rate at which velocity is actually changing is velocity change since last time
                    // over position change since last time
                    double v_rate_actual = deltaVelocity / deltaPosition;
                    // scale the power to achieve the required rate
                    // make sure v_rate_actual is not zero
                    if (v_rate_actual == 0) {
                        v_rate_actual = v_rate_required;
                    }
                    double powerAdjustment = Math.abs(v_rate_required / v_rate_actual);
                    // limit power adjustment within 0.5 and 1.5
                    powerAdjustment = Math.max(powerAdjustment, 0.5);
                    powerAdjustment = Math.min(powerAdjustment, 1.5);
                    power = lastPower * powerAdjustment;
                }
            }
        } else {
            // keep going until power reverses(i.e. passing by the goal mark), then go to next path
            if (signumIsShould != initialPowerSignum) {
                inTargetZone = true;
            }
            double absRampReach = Math.abs(rampReach);
            if (Math.abs(deltaIsShould) > absRampReach) {
                // outside reach value: move with maximum available power
                // for rampReach == 0, the robot will move with maximum power
                // this is an expert mode, the user needs to make sure the robot
                // can stop in time
                power = initialPowerSignum;
            } else {
                if (rampType == RAMPTYPE.LINEAR) {
                    // within reach value: reduce power proportional to distance to goal
                    power = initialPowerSignum*Math.max(minPower/powerScaling,Math.abs(deltaIsShould/rampReach));
                } else {
                    double minPower = 0.2 / powerScaling; // undo scaling for minPower
                    power = signumIsShould * Math.max(minPower,Math.abs(deltaIsShould / absRampReach)); // for the first waitBeforeRamp_ms, use linear ramp
                    if (PathDetails.elapsedTime_ms.milliseconds() > waitBeforeRamp_ms) {
                        double v_abs = Math.max(Math.abs(thisVelocity), minVelocity/2);
                        v_ramp = minVelocity / v_abs;
                        v_ramp = Math.max(v_ramp, 0.9);
                        v_ramp = Math.min(v_ramp, 1.1);
                        power *= v_ramp;
                    }
                }
            }
        }
        if (Math.abs(power) > Math.abs(lastPower) + maxPowerStepUp) { // check if power is increasing too fast
            power = lastPower + signumIsShould * maxPowerStepUp;
        }
        if (dof == THISDOF.Y) {
            yPowerLast = power;
        } else if (dof == THISDOF.X) {
            xPowerLast = power;
        } else {
            turnPowerLast = power;
        }
        return power;
    }
}
