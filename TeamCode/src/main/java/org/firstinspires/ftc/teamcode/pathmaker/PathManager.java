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

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.op.RobotPose;

public class PathManager {
    public static double maxPowerStepUp = 0.05; // this is an addition, balancing power is done later
    public static double powerScaleDown = 0.4; // this is a multiplier making sure zero is the minimum
    private static double powerThreshold = 0.1;
    public static long timeStep_ms = 40;
    public static long PMcycleTime_ms = 0;
    public static double yRampReach_in = 24;
    public static double xRampReach_in = 12;
    public static double turnRampReach_deg = 45;
    public static double yTargetZone_in = 1, xTargetZone_in = 1, turnTargetZone_deg = 1;
    public static double yMinVelocity_inPerSec = 5, xMinVelocity_inPerSec = 5, turnMinVelocity_degPerSec = 5;
    public static double powerScaling = 1;
    public static double yPower, yPowerLast;
    public static double xPower, xPowerLast;
    public static double turnPower, turnPowerLast;
    private enum THISDOF {Y, X, TURN}

    // set path time
    //public static double elapsedTime_ms;
    public static double deltaIsShouldY, deltaIsShouldX, deltaIsShouldAngle;
    public static boolean inTargetZone = false;
    private static ElapsedTime timer = new ElapsedTime();

    public static void moveRobot() throws InterruptedException {
        // initialize
        powerScaling = PathDetails.powerScaling;
        double pathElapsedTime = PathDetails.elapsedTime_ms.milliseconds();
        timer.reset();
        if (PathMakerStateMachine.controlMode == PathMakerStateMachine.ControlMode.AUTONOMOUS) {
            // calculate distance to goal for each DOF, followed by correction power
            // which is proportional to distance to goal but limited by maxPowerStep
            if (pathElapsedTime >= PathDetails.yFieldDelay_ms) {
                deltaIsShouldY = PathDetails.yFieldGoal_in - RobotPose.getFieldY_in();
                yPower = calculateCorrectionPower(THISDOF.Y);
            }
            if (pathElapsedTime >= PathDetails.xFieldDelay_ms) {
                deltaIsShouldX = PathDetails.xFieldGoal_in - RobotPose.getFieldX_in();
                xPower = calculateCorrectionPower(THISDOF.X);
            }
            if (pathElapsedTime >= PathDetails.turnFieldDelay_ms) {
                deltaIsShouldAngle = PathDetails.aFieldGoal_deg - RobotPose.getFieldAngle_deg();
                turnPower = calculateCorrectionPower(THISDOF.TURN);
            }
            if (checkInTargetZone()) {
                // check if robot is in target zone
                // if so, terminate path
                PathMakerStateMachine.terminatePath();
            }
        } else {
            // driver control
            // brake mode
            yPower = PathMakerStateMachine.gamepadY;
            xPower = PathMakerStateMachine.gamepadX;
            turnPower = PathMakerStateMachine.gamepadTurn;
        }
        RobotPose.readPose();
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
        if (powerScaling<1) {
            yPower *= powerScaling;
            xPower *= powerScaling;
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
            deltaIsShould = deltaIsShouldY;
            signumIsShould = Math.signum(deltaIsShouldY);
            rampReach = yRampReach_in;
            lastPower = yPowerLast;
            thisVelocity = RobotPose.getForwardVelocity_inPerSec();
            minVelocity = yMinVelocity_inPerSec;
            initialPowerSignum = PathDetails.yInitialPowerSignum;
        } else if (dof == THISDOF.X) {
            deltaIsShould = deltaIsShouldX;
            signumIsShould = Math.signum(deltaIsShouldX);
            rampReach = xRampReach_in;
            lastPower = xPowerLast;
            thisVelocity = RobotPose.getStrafeVelocity_inPerSec();
            minVelocity = xMinVelocity_inPerSec;
            initialPowerSignum = PathDetails.xInitialPowerSignum;
        } else {
            deltaIsShould = deltaIsShouldAngle;
            signumIsShould = Math.signum(deltaIsShouldAngle);
            rampReach = turnRampReach_deg;
            lastPower = turnPowerLast;
            thisVelocity = RobotPose.getHeadingVelocity_degPerSec();
            minVelocity = turnMinVelocity_degPerSec;
            initialPowerSignum = PathDetails.turnInitialPowerSignum;
        }
        // calculate ramp power
        if (rampReach > 0) {
            // outside reach value: move with maximum available power
            if (Math.abs(deltaIsShould) > rampReach) {
                // outside reach value: move with maximum available power
                // for rampReach == 0, the robot will move with maximum power
                // this is an expert mode, the user needs to make sure the robot
                // can stop in time
                power = signumIsShould;
            } else {
                // within reach value: reduce power proportional to distance to goal
                power = deltaIsShould / rampReach;
            }
        } else {
            // keep going full speed until power reverses, then go to next path
            if (signumIsShould != initialPowerSignum) {
                inTargetZone = true;
            }
            power = initialPowerSignum;
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
