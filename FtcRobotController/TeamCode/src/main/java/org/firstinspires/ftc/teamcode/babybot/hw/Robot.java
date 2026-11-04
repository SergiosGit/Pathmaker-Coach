package org.firstinspires.ftc.teamcode.babybot.hw;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.babybot.math.ControlManager;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Robot {

    public TelemetryManager telemetry;
    public HardwareMap hw;
    public Follower follower;
    public ControlManager controlManager;
    public Drivetrain drivetrain;

    public Robot(OpMode opMode) {
        hw = opMode.hardwareMap;
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.create(hw);

        controlManager = new ControlManager(this, opMode.gamepad1);
        drivetrain = new Drivetrain(this, opMode.gamepad1);
    }
}
