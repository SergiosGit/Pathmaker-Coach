// UpdateTelemetry.java
//
// MIT License
// Copyright (c) 2023 bayrobotics.org
//
package org.firstinspires.ftc.teamcode.op;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configuration.RobotConfig;
import org.firstinspires.ftc.teamcode.pathmaker.PathManager;
import org.firstinspires.ftc.teamcode.pathmaker.RobotPoseSimulation;

@Configurable
public class UpdateTelemetry {

    public static void params(Telemetry telemetry){
        if (RobotConfig.Odometry.odometryType == RobotConfig.Odometry.OdometryType.SIMULATION) {
            telemetry.addData("Sim forward", RobotPoseSimulation.forward);
            telemetry.addData("Sim strafe", RobotPoseSimulation.strafe);
            telemetry.addData("Sim angle", RobotPoseSimulation.angle);
        } else {
            telemetry.addData("forward_in", RobotPose.getFieldY_in());
            telemetry.addData("strafe_in", RobotPose.getFieldX_in());
            telemetry.addData("headingAngle_deg", RobotPose.getFieldAngle_deg());
        }
        telemetry.addData("forwardPower", PathManager.yPower);
        telemetry.addData("strafePower", PathManager.xPower);
        telemetry.addData("turnPower", PathManager.turnPower);
        telemetry.update();
    }
    public static void pose(Telemetry telemetry) {
        RobotPoseSimulation.createPoints();
        telemetry.addData("simulation field Y", RobotPoseSimulation.fieldY);
        telemetry.addData("simulation field X", RobotPoseSimulation.fieldX);
        telemetry.update();
    }
}
