package org.firstinspires.ftc.teamcode.babybot.op.test;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.babybot.hw.Robot;


@TeleOp (name = "Drive test", group = "test")
public class DriveTest extends OpMode {

    protected Robot robot;

    @Override
    public void init() {
        Scheduler.reset();
        robot = new Robot(this);
        robot.drivetrain.setPose(new Pose(0, 0, 0));

        schedule(
                robot.drivetrain.update(),
                robot.controlManager.update()
        );
    }

    @Override
    public void init_loop() {
        Scheduler.execute();
    }

    @Override
    public void loop() {
        Scheduler.execute();
        robot.telemetry.update(this.telemetry);
    }
}

