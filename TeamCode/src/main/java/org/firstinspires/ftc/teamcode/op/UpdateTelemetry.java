// UpdateTelemetry.java
//
// MIT License
// Copyright (c) 2023 bayrobotics.org
//
package org.firstinspires.ftc.teamcode.op;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pathmaker.GameSetup;
import org.firstinspires.ftc.teamcode.pathmaker.PathManager;
import org.firstinspires.ftc.teamcode.pathmaker.RobotPoseSimulation;

@Config
public class UpdateTelemetry {

    public static void params(Telemetry telemetry){
        if (GameSetup.SIMULATION) {
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
    public static void pose(FtcDashboard dashboard) {
        RobotPoseSimulation.createPoints();
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay()
                .setStrokeWidth(1)
                .fillCircle(RobotPoseSimulation.fieldY, RobotPoseSimulation.fieldX, 2)
                .fillPolygon(RobotPoseSimulation.byPoints, RobotPoseSimulation.bxPoints);
        //.setStroke("goldenrod")
        //.strokeCircle(0, 0, ORBITAL_RADIUS)
        //.setFill("black")
        dashboard.sendTelemetryPacket(packet);
    }
}
