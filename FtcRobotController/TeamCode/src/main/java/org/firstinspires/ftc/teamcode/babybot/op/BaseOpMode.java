package org.firstinspires.ftc.teamcode.babybot.op;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.babybot.hw.Robot;


public class BaseOpMode extends OpMode {

    protected Robot robot;

    public void init() {
        Scheduler.reset();
        robot = new Robot(this);
        robot.drivetrain.setPose(new Pose(0, 0, 0));

        schedule(
                robot.drivetrain.update(),
                robot.controlManager.update()
        );
    }

    public void init_loop() {
        Scheduler.execute();
    }

    public void loop() {
        Scheduler.execute();
        robot.telemetry.update(this.telemetry);
    }
}
