package org.firstinspires.ftc.teamcode.babybot.math;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.babybot.hw.Robot;

public class ControlManager {

    private Robot robot;
    private Gamepad gamepad;
    public ControlManager(Robot robot, Gamepad gamepad) {
        this.robot = robot;
        this.gamepad = gamepad;
    }

    public Command update() {
        return infinite(() -> {

        });
    }
}
