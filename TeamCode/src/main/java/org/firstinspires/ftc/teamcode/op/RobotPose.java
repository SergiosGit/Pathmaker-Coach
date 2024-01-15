// RobotPose.java
//
// This method keeps track of where the robot is on the field. It uses the robot centric
// coordinate system (COS) that is defined as Y in forward direction, X in strafe direction, and
// right turns measured in positive degrees.
//
// initializePose:
//      Create COS that is referenced by path parameters defined in PathDetails.
//
// updatePose:
//      apply power calculated in PowerManager towards reaching goals for Y (forward in original COS),
//      X (strafe in original COS) and turn (also with respect to original COS).
//      updatePose maps those powers according to the actual pose of the robot (super-position).
//
// readPose:
//      Reads encoder values and computes the values in inches and degrees. This is only done once
//      per time step.
//
// getHeadingAngle_deg, getForward_in, getStrafe_in:
//      Functions to query the actual pose parameters. These function can be called as much as
//      needed within a time step without actually reading encoder values. They just store the
//      values obtained by the prior readPose() call
//
// MIT License
// Copyright (c) 2023 bayrobotics.org
//
package org.firstinspires.ftc.teamcode.op;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hw.DriveTrain;
import org.firstinspires.ftc.teamcode.hw.MyIMU;
import static org.firstinspires.ftc.teamcode.hw.DriveTrain.getEncoderValues;

import org.firstinspires.ftc.teamcode.pathmaker.PathDetails;
import org.firstinspires.ftc.teamcode.pathmaker.PathManager;

public class RobotPose {
    private static double headingAngle_rad = 0, lastHeadingAngle_rad = 0;
    private static double poseA_deg = 0, lastHeadingAngle_deg = 0;
    private static double forward_in = 0, lastForward_in = 0, strafe_in = 0, lastStrafe_in = 0;
    private static double poseY_in = 0, poseX_in = 0;
    private static double imuAngle_rad = 0, lastImuAngle_rad = 0;
    private static double imuAngle_deg = 0, lastImuAngle_deg = 0;

    private static int currentRightPosition = 0;
    private static int currentLeftPosition = 0;
    private static int currentAuxPosition = 0;

    //to keep track of the previous encoder values
    private static int previousRightPosition = 0;
    private static int previousLeftPosition = 0;
    private static int previousAuxPosition = 0;
    private static int previousStrafeTics = 0;
    private static int previousForwardTics = 0;
    private static int previousTurn_rad = 0;
    public static int currentStrafeTics = 0;
    public static int currentForwardTics = 0;
    public static int currentTurn_rad = 0;

    private static double L; // distance between left and right encoders in cm - LATERAL DISTANCE
    private static double B; // distance between midpoints of left and right encoders and encoder aux
    private static double R; // odometry wheel radius in cm
    private static double N; // REV encoders tic per revolution
    private static double cm_per_tick, cm_per_tick_strafe;

    private static Telemetry poseTelemetry;
    private static DriveTrain poseDriveTrain;
    private static MyIMU imu = new MyIMU(null);
    public enum ODOMETRY {DEADWHEEL, XYPLUSIMU};
    public static ODOMETRY odometry;
    public static int[] encoderValues = new int[4];
    public static double[] motorCurrents = new double[4];
    public static double[] motorVelocities = new double[4];

    public static void initializePose(LinearOpMode opMode, DriveTrain driveTrain, Telemetry telemetry) throws InterruptedException {
        driveTrain.init();
        odometry = ODOMETRY.XYPLUSIMU;
        imu.setOpMode(opMode);
        imu.init(opMode);
        imu.resetAngle();
        imuAngle_rad = imu.getAngle_rad();
        imuAngle_deg = imu.thisAngle_deg; // initialized after call getAngle_rad()
        poseTelemetry = telemetry;
        poseDriveTrain = driveTrain;
        readPose();
        headingAngle_rad = 0; // PathDetails.turnOffset_deg / 180. * Math.PI;
        lastHeadingAngle_rad = 0;
        poseA_deg = 0;
        lastHeadingAngle_deg = 0;
        forward_in = 0;
        lastForward_in = 0;
        strafe_in = 0;
        lastStrafe_in = 0;
        poseY_in = getFieldY_in(); // PathDetails.forwardOffset_in;
        poseX_in = getFieldX_in(); // PathDetails.strafeOffset_in;
        currentAuxPosition = 0;
        currentRightPosition = 0;
        currentLeftPosition = 0;
        previousAuxPosition = 0;
        previousRightPosition = 0;
        previousLeftPosition = 0;
        PathDetails.aFieldGoal_deg = 0;
        PathDetails.yFieldGoal_in = 0;
        PathDetails.xFieldGoal_in = 0;
        PathDetails.lastTurnGoal = 0;
        if (odometry == ODOMETRY.DEADWHEEL) {
            L = 32.9438; // distance between left and right encoders in cm - LATERAL DISTANCE
            B = 11.724; // distance between midpoints of left and right encoders and encoder aux
            R = 1.9; // odometry wheel radius in cm
            N = 8192; // REV encoders tic per revolution
        } else if (odometry == ODOMETRY.XYPLUSIMU) {
            R = 4.8; // Mecanum wheel radius in cm (OD=96mm)
            N = 537.7; // 312 RPM motor encoder tics per revolution (PPR)
            cm_per_tick_strafe = 73.2/1477; // measured with coach chassis
        } else {
            R = 1;
            N = 1;
        }
        cm_per_tick = (2.0 * Math.PI * R)/N;
    }
    public static void updatePose(double forwardDrive, double strafeDrive, double rotateDrive){
        double powerFL, powerFR, powerBL, powerBR;
        double radians = 0;
        // get the angle in which the robot is actually headed
        radians = getFieldAngle_rad();
        // project the desired forwardDrive (which is relative to the original
        // forward direction in the coordinate system at the beginning of the
        // path) onto the the actual drive train. This will not change the
        // maximum power seen at any of the Mecanum wheels
        double forwardHeading = forwardDrive * Math.cos(radians) + strafeDrive * Math.sin(radians);
        double strafeHeading = -forwardDrive * Math.sin(radians) + strafeDrive * Math.cos(radians);
        // now add the power components for the drive train
        // forward power is the same on all wheels
        powerFL = forwardHeading; powerFR = forwardHeading;
        powerBL = forwardHeading; powerBR = forwardHeading;
        // add strafe power
        powerFL += strafeHeading; powerFR -= strafeHeading;
        powerBL -= strafeHeading; powerBR += strafeHeading;
        // add turn power
        powerFL += rotateDrive; powerFR -= rotateDrive;
        powerBL += rotateDrive; powerBR -= rotateDrive;
        poseDriveTrain.setMotorPowers(powerFL,powerBL,powerBR,powerFR);
    }
    public static void readPose() {
        double dtheta, dx_in, dy_in, dx, dy, x, y;
        int dn1, dn2, dn3, dn4;

        // Clear the BulkCache once at the beginning of each control cycle
        for (LynxModule module : DriveTrain.allHubs) {
            module.clearBulkCache();
        }

        encoderValues = getEncoderValues();
        motorCurrents = DriveTrain.getMotorCurrents();
        motorVelocities = DriveTrain.getMotorVelocities();

        if (odometry == ODOMETRY.DEADWHEEL) {
            currentRightPosition = encoderValues[0];
            currentLeftPosition = encoderValues[2];
            currentAuxPosition = encoderValues[1];

            dn1 = currentLeftPosition - previousLeftPosition;
            dn2 = currentRightPosition - previousRightPosition;
            dn3 = currentAuxPosition - previousAuxPosition;

            //find out robot movement in cm
            dtheta = cm_per_tick * (dn2 - dn1) / L;
            dy = cm_per_tick * (dn1 + dn2) / 2.0;
            dx = cm_per_tick * (-dn3 + (dn2 - dn1) * B / L);
            lastHeadingAngle_rad = headingAngle_rad;
            headingAngle_rad += dtheta;
        } else if (odometry == ODOMETRY.XYPLUSIMU){
            lastHeadingAngle_rad = headingAngle_rad;
            headingAngle_rad = imu.getAngle_rad();
            // dy is the average of all 4 encoders
            currentForwardTics = (encoderValues[0] + encoderValues[1] + encoderValues[2] + encoderValues[3]) / 4;
            currentStrafeTics = (encoderValues[0] - encoderValues[1] + encoderValues[2] - encoderValues[3]) / 4;
            dx = (currentStrafeTics - previousStrafeTics) * cm_per_tick_strafe;
            dy = (currentForwardTics - previousForwardTics) * cm_per_tick;
            previousStrafeTics = currentStrafeTics;
            previousForwardTics = currentForwardTics;
        } else {
            dx = 0;
            dy = 0;
            forward_in = 0;
            strafe_in = 0;
            headingAngle_rad = 0;
        }
        dx_in = dx / 2.54;
        dy_in = dy / 2.54;

        lastForward_in = forward_in;
        forward_in += dy_in;

        lastStrafe_in = strafe_in;
        strafe_in += dx_in;

        previousLeftPosition = currentLeftPosition;
        previousRightPosition = currentRightPosition;
        previousAuxPosition = currentAuxPosition;

        // book keeping for forward and strafe direction movement in the
        // robot coordinate system (COS at start)
        double deltaPathForward_in = forward_in - lastForward_in;
        double deltaPathStrafe_in = strafe_in - lastStrafe_in;
        double sin = Math.sin(headingAngle_rad);
        double cos = Math.cos(headingAngle_rad);
        poseY_in += deltaPathForward_in * cos - deltaPathStrafe_in * sin;
        poseX_in += deltaPathStrafe_in * cos + deltaPathForward_in * sin;
    }
    public static double getFieldAngle_rad(){
        // call readPose first (but only once for all encoders, imu)
        return headingAngle_rad + PathDetails.aFieldOffset_deg / 180. * Math.PI;
    }
    public static double getFieldAngle_deg(){

        return headingAngle_rad / Math.PI * 180 + PathDetails.aFieldOffset_deg;
    }
    public static double getIMUAngle_rad() {
        return imuAngle_rad;
    }
    public static double getFieldY_in(){
        // call readPose first (but only once for all encoders, imu)
        // get actual forward position of the robot in the coordinate system
        // defined at the beginning of the path.
        return poseY_in + PathDetails.yFieldOffset_in;
    }
    public static double getFieldX_in(){
        // call readPose first (but only once for all encoders, imu)
        // get actual strafe (lateral) position of the robot in the
        // coordinate system defined at the beginning of the path
        return poseX_in + PathDetails.xFieldOffset_in;
    }
    public static double getForwardVelocity_inPerSec(){
        // call readPose first (but only once for all encoders, imu)
        // get actual forward velocity of the robot in the coordinate system
        // defined at the beginning of the path.
        return (forward_in - lastForward_in) / PathManager.timeStep_ms * 1000;
    }
    public static double getStrafeVelocity_inPerSec(){
        // call readPose first (but only once for all encoders, imu)
        // get actual strafe (lateral) velocity of the robot in the
        // coordinate system defined at the beginning of the path
        return (strafe_in - lastStrafe_in) / PathManager.timeStep_ms * 1000;
    }
    public static double getHeadingVelocity_degPerSec(){
        // call readPose first (but only once for all encoders, imu)
        // get actual heading velocity of the robot in the
        // coordinate system defined at the beginning of the path
        return (headingAngle_rad - lastHeadingAngle_rad) / PathManager.timeStep_ms * 1000;
    }

    public static boolean isRobotAtRest() {
        return (Math.abs(getForwardVelocity_inPerSec()) < 0.1 &&
                Math.abs(getStrafeVelocity_inPerSec()) < 0.1 &&
                Math.abs(getHeadingVelocity_degPerSec()) < 0.1);
    }

    public static double [] rebaseRelativeToTag(double tagX, double tagY, double tagAngle, int tagID) {
        // rebase robot pose based on tag identification
        double tagXYA [] = tagOffset(tagID);
        setPose(tagXYA[1] - tagX, tagXYA[0] - tagY, tagXYA[2] - tagAngle);
        return tagXYA;
    }
    public static void setPose(double X_in, double Y_in, double A_deg) {
        strafe_in = lastStrafe_in = poseX_in = X_in;
        forward_in = lastForward_in = poseY_in = Y_in;
        lastHeadingAngle_rad = headingAngle_rad = A_deg / 180. * Math.PI;
    }

    public static double [] tagOffset(int tagID) {
        // rebase robot pose based on tag identification
        tagID =  Math.max(Math.min(tagID, 10), 0);
        double [] YOffset_in = {0, 62, 62, 62,62,62,62,-72.5,-72.5,-72.5,-72.5};
        double [] XOffset_in = {0,-42,-36,-30,30,36,42, 44.5,   36,  -12,-44.5}; // for testing only (#9 actual: X=-36)
        double [] AOffset_deg ={0,  0,  0,  0, 0, 0, 0,    0,    0,    0,    0};
        return new double[]{YOffset_in[tagID], XOffset_in[tagID], AOffset_deg[tagID]};
    }
}
