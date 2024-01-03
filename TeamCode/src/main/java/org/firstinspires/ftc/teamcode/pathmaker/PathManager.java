//
// PathManager.java
//
// This method calculates the motor powers to drive the robot along the path. The use of
// battery power is maximized for each DOF. For example, if the path only consists of
// forward motion, the absolute power applied will be 1. However, if forward, strafe, and turn
// are all applied, the absolute power for each DOF will be 1/3.
// Power transitions are ramped with a maximum change of maxPowerStep each timeStep_ms.
// When the robot is within "reach" of the goal, the power is ramped down to 0. The reach
// in each DOF is defined by forwardRampReach_in, strafeRampReach_in, and turnRampReach_deg,
// respectively.
// The path will terminate after pathTime_ms defined in PathDetails without applying
// a ramp-down of power.
//
// MIT License
// Copyright (c) 2023 bayrobotics.org
//


package org.firstinspires.ftc.teamcode.pathmaker;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.op.RobotPose;
// on Android, use the following import to run the pathmaker in real mode:
// import org.firstinspires.ftc.teamcode.op.RobotPose;

public class PathManager {
    static double maxPowerStep = 0.05;
    public static long timeStep_ms = 40;
    public static long PMcycleTime_ms = 0;
    public static double forwardRampReach_in = 24;
    public static double strafeRampReach_in = 12;
    public static double turnRampReach_deg = 45;
    public static double forwardTargetZone_in = 1;
    public static double strafeTargetZone_in = 1;
    public static double turnTargetZone_deg = 2;
    public static double forwardMinVelocity_inPerSec = 5;
    public static double strafeMinVelocity_inPerSec = 5;
    public static double turnMinVelocity_degPerSec = 5;
    public static double powerScaling = 1;
    public static double forwardPower, forwardPowerLast;
    public static double strafePower, strafePowerLast;
    public static double turnPower, turnPowerLast;
    private enum THISDOF {FORWARD, STRAFE, TURN}

    // set path time
    //public static double elapsedTime_ms;
    private static double deltaIsShouldForward, deltaIsShouldStrafe, deltaIsShouldAngle;
    public static boolean inTargetZone = false;
    private static ElapsedTime timer = new ElapsedTime();

    public static void moveRobot() throws InterruptedException {
        // initialize
        powerScaling = PathDetails.powerScaling;
        double pathElapsedTime = PathDetails.elapsedTime_ms.milliseconds();
        timer.reset();
        // calculate distance to goal for each DOF, followed by correction power
        // which is proportional to distance to goal but limited by maxPowerStep
        if (pathElapsedTime >= PathDetails.forwardDelay_ms) {
            deltaIsShouldForward = calculateIsShould(THISDOF.FORWARD);
            forwardPower = calculateCorrectionPower(THISDOF.FORWARD);
        }
        if (pathElapsedTime >= PathDetails.strafeDelay_ms) {
            deltaIsShouldStrafe = calculateIsShould(THISDOF.STRAFE);
            strafePower = calculateCorrectionPower(THISDOF.STRAFE);
        }
        if (pathElapsedTime >= PathDetails.turnDelay_ms) {
            deltaIsShouldAngle = calculateIsShould(THISDOF.TURN);
            turnPower = calculateCorrectionPower(THISDOF.TURN);
        }
        balancePower(); // balance power so it doesn't exceed 1
        RobotPose.readPose();
        if (checkInTargetZone()) {
            // check if robot is in target zone
            // if so, terminate path
            PathMakerStateMachine.terminatePath();
        }
        RobotPose.updatePose(forwardPower, strafePower, turnPower);  // move robot
        // sleep the time needed to match timeStep_ms
        PMcycleTime_ms = (long) timer.milliseconds() + 1; // 1 ms overhead
        if (PMcycleTime_ms<timeStep_ms) {
            // make sure to wait at least timeStep_ms before next iteration
            // timeStep could be exceeded when sensor data is read too slow (readPose)
            Thread.sleep(timeStep_ms-PMcycleTime_ms);
        }
    }
    private static boolean checkInTargetZone() {
        // check if robot is in target zone
        if (Math.abs(deltaIsShouldForward) < forwardTargetZone_in &&
                Math.abs(deltaIsShouldStrafe) < strafeTargetZone_in &&
                Math.abs(deltaIsShouldAngle) < turnTargetZone_deg) {
            inTargetZone = true;
        } else
            inTargetZone = false;
        return inTargetZone;
    }
    private static void balancePower() {
        // balance power so it doesn't exceed 1
        // remember max |power| is 1 but will be distributed evenly on the 3 DOFs
        double sumPower = Math.abs(forwardPower) + Math.abs(strafePower) + Math.abs(turnPower);
        if (sumPower != 0 & sumPower > 1) {
            forwardPower /= sumPower;
            strafePower /= sumPower;
            turnPower /= sumPower;
        }
        // power scaling only for forward and strafe
        forwardPower *= powerScaling;
        strafePower *= powerScaling;
    }

    private static double calculateCorrectionPower(THISDOF dof) {
        double rampReach;
        double deltaIsShould;
        double power;
        double signumIsShould;
        double lastPower;
        double thisVelocity, minVelocity;
        if (dof == THISDOF.FORWARD) {
            deltaIsShould = deltaIsShouldForward;
            signumIsShould = Math.signum(deltaIsShouldForward);
            rampReach = forwardRampReach_in;
            lastPower = forwardPowerLast;
            thisVelocity = RobotPose.getForwardVelocity_inPerSec();
            minVelocity = forwardMinVelocity_inPerSec;
        } else if (dof == THISDOF.STRAFE) {
            deltaIsShould = deltaIsShouldStrafe;
            signumIsShould = Math.signum(deltaIsShouldStrafe);
            rampReach = strafeRampReach_in;
            lastPower = strafePowerLast;
            thisVelocity = RobotPose.getStrafeVelocity_inPerSec();
            minVelocity = strafeMinVelocity_inPerSec;
        } else {
            deltaIsShould = deltaIsShouldAngle;
            signumIsShould = Math.signum(deltaIsShouldAngle);
            rampReach = turnRampReach_deg;
            lastPower = turnPowerLast;
            thisVelocity = RobotPose.getHeadingVelocity_degPerSec();
            minVelocity = turnMinVelocity_degPerSec;
        }
        // calculate ramp power
        if (Math.abs(deltaIsShould) > rampReach || rampReach == 0) {
            // outside reach value: move with maximum available power
            // for rampReach == 0, the robot will move with maximum power
            // this is an expert mode, the user needs to make sure the robot
            // can stop in time
            power = signumIsShould;
        } else {
            // within reach value: reduce power proportional to distance
            power = deltaIsShould / rampReach;
        }
        // check if power is increasing too fast
        if (Math.abs(power) > Math.abs(lastPower) + maxPowerStep) {
            power = lastPower + signumIsShould * maxPowerStep;
        }
        if (dof == THISDOF.FORWARD) {
            forwardPowerLast = power;
        } else if (dof == THISDOF.STRAFE) {
            strafePowerLast = power;
        } else {
            turnPowerLast = power;
        }
        return power;
    }

    private static double calculateIsShould(THISDOF dof) {
        if (dof == THISDOF.FORWARD) {
            return PathDetails.forwardGoal_in - RobotPose.getForward_in();
        } else if (dof == THISDOF.STRAFE) {
            return PathDetails.strafeGoal_in - RobotPose.getStrafe_in();
        } else {
            return PathDetails.turnGoal_deg - RobotPose.getHeadingAngle_deg();
        }
    }
}
