package org.firstinspires.ftc.teamcode.hw;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.configuration.ShooterConfig;

public class ExtraStuff {
    //Intake Code:
    public static DcMotorEx intakeMotor;
    public static Servo intakeServo;
    public static DcMotorEx outtakeMotor;
    public static Shooter shooter;
    public static double targetRpm = 0; // real wheel speed in RPM, no longer a motor power

    public ExtraStuff(DcMotorEx intakeMotor, DcMotorEx outtakeMotor, Servo intakeServo) {
        ExtraStuff.intakeMotor = intakeMotor;
        ExtraStuff.outtakeMotor = outtakeMotor;
        ExtraStuff.intakeServo = intakeServo;
    }

    /** Call once during init, after creating ExtraStuff with the real motors. */
    public static void initShooter(HardwareMap hardwareMap) {
        // Static values survive from one OpMode to the next, so start from a known state.
        targetRpm = 0;
        shooter = new Shooter(hardwareMap, outtakeMotor);
    }

    /** Used by autonomous. The wheel speed is held by shooter.update() in PathMakerStateMachine. */
    public static void Extra_Stuff(double targetRpm, double intakePower, double intakeServoPosition) {
        intakeMotor.setPower(intakePower);
        intakeServo.setPosition(intakeServoPosition);
        ExtraStuff.targetRpm = targetRpm;
        shooter.setTargetRpm(targetRpm);
    }

    public void getGamepadInput(Gamepad gamepad, Gamepad gamepad2) {
        int leftBumper = 0;
        if (gamepad.left_bumper) {
            leftBumper = 1;
        }
        if (gamepad2.left_trigger > 0.5) {
            // Optional: hold the ball back until the wheel has recovered from the previous shot
            boolean waitForWheel = ShooterConfig.gateIntakeOnSpeed && shooter.isOn() && !shooter.isAtSpeed();
            intakeMotor.setPower(waitForWheel ? 0 : 0.7 + (-1.4 * leftBumper));
        } else {
            intakeMotor.setPower(-0.25);
        }
        if (gamepad2.dpadDownWasReleased()) {
            targetRpm = Math.max(0, targetRpm - ShooterConfig.dpadStepRpm);
        }
        else if (gamepad2.dpadUpWasReleased()) {
            targetRpm += ShooterConfig.dpadStepRpm;
        }
        if (gamepad2.dpadLeftWasReleased())
        {
            intakeServo.setPosition(0);
        }
        else if (gamepad2.dpadRightWasReleased())
        {
            intakeServo.setPosition(1);
        }
        if (gamepad2.xWasPressed()) {
            targetRpm = ShooterConfig.presetLowRpm;
        }
        if (gamepad2.yWasPressed()) {
            targetRpm = ShooterConfig.presetMidRpm;
        }
        if (gamepad2.bWasPressed()) {
            targetRpm = ShooterConfig.presetHighRpm;
        }
        if (gamepad2.rightBumperWasPressed()) {
            targetRpm = 0; // shooter off, the wheel coasts down
        }
        if (gamepad2.a) {
            // Hold A to clear a jam. Letting go returns to the speed we had before.
            shooter.setOpenLoopPower(ShooterConfig.reversePower);
        } else {
            shooter.setTargetRpm(targetRpm);
        }
        shooter.update();
        if (shooter.justBecameReady()) {
            gamepad2.rumble(150); // tell the driver the wheel is ready
        }
    }
}
