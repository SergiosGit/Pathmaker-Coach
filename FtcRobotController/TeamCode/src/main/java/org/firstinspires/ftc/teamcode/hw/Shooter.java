package org.firstinspires.ftc.teamcode.hw;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configuration.ShooterConfig;

import java.util.Iterator;

/**
 * Connects the speed controller to the real robot: reads the wheel speed from the OctoQuad, runs
 * VelocityPidController and sends the result to the motor.
 *
 * How to use it:
 *   shooter.setTargetRpm(4500);          // whenever you want a new speed
 *   shooter.update();                    // exactly once per loop, every loop
 *   if (shooter.isAtSpeed()) { ... }     // safe to feed a ball
 *   shooter.addTelemetry(telemetry);     // before telemetry.update()
 *
 * The motor only changes power inside update(). If the loop stops calling it, the motor keeps its
 * last power, so never put a long sleep() in a loop while the shooter is running.
 *
 * If the encoder fails (OctoQuad missing, cable out, encoder running backwards), the shooter does
 * not push full power into a wheel it cannot see. It falls back to feedforward only, which works
 * about like the old fixed-power code, and reports the problem on the telemetry.
 */
public class Shooter {

    private enum Mode { OFF, VELOCITY, OPEN_LOOP }

    private static final double VOLTAGE_READ_INTERVAL_MS = 250;
    private static final double MIN_POWER_CHANGE = 0.005;

    private final DcMotorEx motor;
    private final VelocityPidController pid = new VelocityPidController();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime voltageTimer = new ElapsedTime();

    private OctoQuad octoQuad;
    private VoltageSensor voltageSensor;
    private int requestedWindowMs;
    private int appliedWindowMs;

    private Mode mode = Mode.OFF;
    private double targetRpm = 0;
    private double openLoopPower = 0;

    private double measuredRpm = 0;
    private double batteryVolts = ShooterConfig.nominalVolts;
    private double power = 0;
    private double lastSentPower = 0;
    private double loopMs = 0;

    private String fault = null;
    private double noSpeedMs = 0;

    private double inToleranceMs = 0;
    private boolean atSpeed = false;
    private boolean justBecameReady = false;

    private boolean atSpeedBeforeThisDip = false;
    private boolean dipInProgress = false;
    private int shotCount = 0;
    private double dipRpm = 0;
    private double dipElapsedMs = 0;
    private double lastDipRpm = 0;
    private double lastRecoveryMs = 0;

    /** @param motor the shooter motor, already fetched from the hardwareMap */
    public Shooter(HardwareMap hardwareMap, DcMotorEx motor) {
        this.motor = motor;

        // We run our own controller, so the hub must pass our power straight through to the motor.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // FLOAT lets the heavy flywheel coast down. BRAKE would stop it violently and stress the motor.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(ShooterConfig.reverseMotor
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setPower(0);

        Iterator<VoltageSensor> sensors = hardwareMap.voltageSensor.iterator();
        voltageSensor = sensors.hasNext() ? sensors.next() : null;

        try {
            octoQuad = hardwareMap.tryGet(OctoQuad.class, ShooterConfig.octoQuadName);
            if (octoQuad == null) {
                fault = "no device called '" + ShooterConfig.octoQuadName + "' in the robot configuration";
            } else {
                configureOctoQuad();
            }
        } catch (RuntimeException e) {
            octoQuad = null;
            fault = "OctoQuad did not respond: " + e.getMessage();
        }
        loopTimer.reset();
    }

    private void configureOctoQuad() {
        // We leave the rest of the board's settings alone and do not write them to flash: they are
        // set again at every start, so nothing can be left over from a previous program.
        octoQuad.setChannelBankConfig(OctoQuad.ChannelBankConfig.ALL_QUADRATURE);
        octoQuad.setSingleEncoderDirection(ShooterConfig.encoderChannel, ShooterConfig.reverseEncoder
                ? OctoQuad.EncoderDirection.REVERSE : OctoQuad.EncoderDirection.FORWARD);
        applyVelocityWindow();
    }

    private void applyVelocityWindow() {
        requestedWindowMs = ShooterConfig.velocityWindowMs;
        appliedWindowMs = Math.max(OctoQuad.MIN_VELOCITY_MEASUREMENT_INTERVAL_MS,
                Math.min(longestSafeWindowMs(), requestedWindowMs));
        octoQuad.setSingleVelocitySampleInterval(ShooterConfig.encoderChannel, appliedWindowMs);
    }

    /**
     * The OctoQuad stores counts-per-window in a 16-bit register, which holds at most 32767. With a
     * high-resolution encoder a long window can overflow it: at 8192 counts per revolution, a 40 ms
     * window overflows at 6000 RPM. The value wraps around to a wrong number that can still look
     * believable, so we prevent that by never using a window that could overflow.
     */
    private static int longestSafeWindowMs() {
        double countsPerMs = ShooterConfig.maxPlausibleRpm * ShooterConfig.ENCODER_COUNTS_PER_REV / 60000.0;
        return Math.min(OctoQuad.MAX_VELOCITY_MEASUREMENT_INTERVAL_MS, (int) (Short.MAX_VALUE / countsPerMs));
    }

    /** Hold this wheel speed. Zero or less switches the shooter off and lets the wheel coast. */
    public void setTargetRpm(double rpm) {
        if (rpm > 0) {
            mode = Mode.VELOCITY;
            targetRpm = rpm;
        } else {
            stop();
        }
    }

    public void stop() {
        mode = Mode.OFF;
        targetRpm = 0;
    }

    /** Send a fixed power with no speed control: for clearing jams and for measuring kV. */
    public void setOpenLoopPower(double newPower) {
        mode = Mode.OPEN_LOOP;
        targetRpm = 0;
        openLoopPower = newPower;
    }

    /** Call exactly once per loop, every loop, while the OpMode runs. */
    public void update() {
        double dtSeconds = loopTimer.seconds();
        loopTimer.reset();
        loopMs = dtSeconds * 1000;

        readSpeed();
        readBatteryIfDue();

        pid.setLimits(ShooterConfig.iZoneRpm, ShooterConfig.iMaxPower,
                ShooterConfig.derivativeFilterAlpha, ShooterConfig.nominalVolts);

        switch (mode) {
            case VELOCITY:
                checkForEncoderFault();
                if (fault == null) {
                    pid.setGains(ShooterConfig.kS, ShooterConfig.kV,
                            ShooterConfig.kP, ShooterConfig.kI, ShooterConfig.kD);
                    power = pid.update(targetRpm, measuredRpm, dtSeconds, batteryVolts);
                } else {
                    // We cannot trust the speed reading, so PID is off and only the feedforward runs.
                    pid.setGains(ShooterConfig.kS, ShooterConfig.kV, 0, 0, 0);
                    power = pid.update(targetRpm, targetRpm, dtSeconds, batteryVolts);
                }
                break;
            case OPEN_LOOP:
                pid.reset();
                // Reversing a fast flywheel draws a huge current, so wait until it has slowed down.
                boolean tooFastToReverse = openLoopPower < 0 && measuredRpm > ShooterConfig.reverseMaxRpm;
                power = tooFastToReverse ? 0 : openLoopPower;
                break;
            default:
                pid.update(0, measuredRpm, dtSeconds, batteryVolts);
                power = 0;
                break;
        }

        // Each motor command costs loop time on the hub, so skip changes too small to matter.
        boolean mustTurnOff = power == 0 && lastSentPower != 0;
        if (mustTurnOff || Math.abs(power - lastSentPower) >= MIN_POWER_CHANGE) {
            motor.setPower(power);
            lastSentPower = power;
        }

        updateAtSpeed(dtSeconds);
        updateShotDetection(dtSeconds);
    }

    /**
     * The OctoQuad counts encoder steps during a fixed time window, using its own clock, and stores
     * the result. So the reading stays accurate even though our loop time varies from 20 to 40 ms.
     */
    private void readSpeed() {
        if (octoQuad == null) {
            return;
        }
        if (ShooterConfig.velocityWindowMs != requestedWindowMs) {
            applyVelocityWindow();  // somebody changed the window in Panels
        }
        double countsPerWindow = octoQuad.readSingleVelocity(ShooterConfig.encoderChannel);
        double revolutions = countsPerWindow / ShooterConfig.ENCODER_COUNTS_PER_REV;
        double rpm = revolutions * (60000.0 / appliedWindowMs);

        // A single I2C read has no error check. A speed the motor cannot reach must be a bad reading.
        if (Math.abs(rpm) <= ShooterConfig.maxPlausibleRpm) {
            measuredRpm = rpm;
        }
    }

    /** Total encoder count, for checking the wiring by hand. An extra I2C read, so not for match code. */
    public int readEncoderCounts() {
        return octoQuad == null ? 0 : octoQuad.readSinglePosition(ShooterConfig.encoderChannel);
    }

    /** Reading the battery is slow compared to the rest of the loop and it changes slowly: read rarely. */
    private void readBatteryIfDue() {
        if (voltageSensor != null && voltageTimer.milliseconds() >= VOLTAGE_READ_INTERVAL_MS) {
            voltageTimer.reset();
            double volts = voltageSensor.getVoltage();
            if (volts > 1) {
                batteryVolts = volts;
            }
        }
    }

    /** Lots of power but no forward speed means the speed reading is wrong: cable, channel or direction. */
    private void checkForEncoderFault() {
        if (fault != null) {
            return;
        }
        if (lastSentPower > 0.3 && measuredRpm < 100) {
            noSpeedMs += loopMs;
            if (noSpeedMs >= ShooterConfig.encoderFaultMs) {
                fault = "power on but encoder reads " + Math.round(measuredRpm)
                        + " RPM. Check cable, channel and reverseEncoder";
                pid.reset();
            }
        } else {
            noSpeedMs = 0;
        }
    }

    private void updateAtSpeed(double dtSeconds) {
        boolean wasAtSpeed = atSpeed;
        boolean inTolerance = mode == Mode.VELOCITY && fault == null
                && Math.abs(targetRpm - measuredRpm) <= ShooterConfig.atSpeedToleranceRpm;
        inToleranceMs = inTolerance ? inToleranceMs + dtSeconds * 1000 : 0;
        atSpeed = inToleranceMs >= ShooterConfig.atSpeedHoldMs;
        justBecameReady = atSpeed && !wasAtSpeed;
    }

    /** A sudden speed drop from "at speed" is a ball going through. We record how deep and how long. */
    private void updateShotDetection(double dtSeconds) {
        if (mode != Mode.VELOCITY || fault != null) {
            dipInProgress = false;
            return;
        }
        double errorRpm = targetRpm - measuredRpm;
        if (!dipInProgress) {
            if (atSpeedBeforeThisDip && errorRpm >= ShooterConfig.shotDipRpm) {
                dipInProgress = true;
                shotCount++;
                dipRpm = errorRpm;
                dipElapsedMs = 0;
            }
        } else {
            dipElapsedMs += dtSeconds * 1000;
            dipRpm = Math.max(dipRpm, errorRpm);
            if (errorRpm <= ShooterConfig.atSpeedToleranceRpm) {
                dipInProgress = false;
                lastDipRpm = dipRpm;
                lastRecoveryMs = dipElapsedMs;
            }
        }
        // Remember "was close to target", so that the first spin-up is not counted as a shot.
        atSpeedBeforeThisDip = Math.abs(errorRpm) <= ShooterConfig.atSpeedToleranceRpm
                || (atSpeedBeforeThisDip && errorRpm < ShooterConfig.shotDipRpm);
    }

    /** True when the wheel has been on target long enough that a ball fed now will fly true. */
    public boolean isAtSpeed() { return atSpeed; }

    /** True for one loop when the wheel reaches speed. Handy for a gamepad rumble. */
    public boolean justBecameReady() { return justBecameReady; }

    public boolean isOn() { return mode == Mode.VELOCITY; }
    public double getTargetRpm() { return targetRpm; }
    public double getMeasuredRpm() { return measuredRpm; }
    public double getPower() { return power; }
    public double getBatteryVolts() { return batteryVolts; }
    public int getAppliedWindowMs() { return appliedWindowMs; }
    public String getFault() { return fault; }

    /**
     * Panels' Graph tab plots any value published as "name: number". In the graph, choose
     * shooter_target_rpm + shooter_measured_rpm for the main picture, then look at shooter_power and
     * the term_ values together in a second graph, because they use a very different scale.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("shooter_target_rpm", targetRpm);
        telemetry.addData("shooter_measured_rpm", measuredRpm);
        telemetry.addData("shooter_error_rpm", targetRpm - measuredRpm);
        telemetry.addData("shooter_power", power);
        telemetry.addData("term_feedforward", pid.getFeedforwardTerm());
        telemetry.addData("term_p", pid.getProportionalTerm());
        telemetry.addData("term_i", pid.getIntegralTerm());
        telemetry.addData("term_d", pid.getDerivativeTerm());
        telemetry.addData("shooter_at_speed", atSpeed ? 1 : 0);
        telemetry.addData("shooter_shots", shotCount);
        telemetry.addData("shooter_last_dip_rpm", lastDipRpm);
        telemetry.addData("shooter_last_recovery_ms", lastRecoveryMs);
        telemetry.addData("shooter_loop_ms", loopMs);
        telemetry.addData("battery_volts", batteryVolts);
        if (fault != null) {
            telemetry.addLine("SHOOTER FAULT, using fixed power: " + fault);
        }
    }
}
