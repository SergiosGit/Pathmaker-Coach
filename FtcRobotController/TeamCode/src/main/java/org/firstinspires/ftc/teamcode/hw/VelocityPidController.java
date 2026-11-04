package org.firstinspires.ftc.teamcode.hw;

/**
 * Feedforward + PID speed controller for the shooter flywheel.
 *
 * This class is plain Java: it knows nothing about motors, the OctoQuad or Panels. You hand it
 * numbers and it hands back a motor power. That makes it possible to test it on a laptop, which is
 * exactly what docs/shooter_sim/ShooterSim.java does.
 *
 * The output is the sum of five terms. All of them are in "motor power" units (0 to 1):
 *
 *   kS            just enough power to overcome friction in the shooter
 *   kV * target   the power that holds this speed in steady state  (the "feedforward", our best guess)
 *   kP * error    push harder the further we are from the target   (reacts to a ball hitting the wheel)
 *   integral      slowly removes any small error that is left over (corrects a slightly wrong kV)
 *   kD * dRPM/dt  resists fast speed changes (off by default, see below)
 *
 * Feedforward does about 90% of the work, because we can calculate what power a speed needs.
 * PID only has to correct what the guess gets wrong, and that is why it can stay gentle.
 *
 * Three details that separate a robot controller from the textbook formula:
 *
 *  1. Anti-windup. A ball can pull the wheel 300 RPM low with the motor already at full power.
 *     If the integral kept growing during that time, the wheel would overshoot afterwards and the
 *     next ball would fly long. So we only integrate when close to the target (iZoneRpm), and never
 *     while the output is stuck against its limit.
 *
 *  2. Derivative on measurement. Using the measured speed instead of the error means that changing
 *     the target does not cause a sudden jump in power.
 *
 *  3. Derivative is off by default (kD = 0). The flywheel is so heavy that it does not need damping,
 *     and in simulation kD only made the motor power noisier. The term is here to experiment with.
 */
public class VelocityPidController {

    private double kS, kV, kP, kI, kD;
    private double iZoneRpm = 300;
    private double iMaxPower = 0.15;
    private double derivativeFilterAlpha = 0.3;
    private double nominalVolts = 12.0;

    // The motor must never brake the flywheel by pushing backwards: that makes huge current spikes.
    // To slow down, we cut the power and let friction do the work.
    private double minOutput = 0.0;
    private double maxOutput = 1.0;

    private double integral = 0;
    private double filteredRpmPerSec = 0;
    private double lastMeasuredRpm = 0;
    private boolean hasLastMeasurement = false;

    private double feedforwardTerm, proportionalTerm, derivativeTerm, output;

    /** Called every loop so that values changed in Panels take effect immediately. */
    public void setGains(double kS, double kV, double kP, double kI, double kD) {
        this.kS = kS;
        this.kV = kV;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * @param iZoneRpm only integrate when the error is smaller than this
     * @param iMaxPower the integral can never contribute more than this much power
     * @param derivativeFilterAlpha 0 to 1. Smaller means a smoother but slower derivative
     * @param nominalVolts battery voltage at which kS and kV were measured
     */
    public void setLimits(double iZoneRpm, double iMaxPower, double derivativeFilterAlpha, double nominalVolts) {
        this.iZoneRpm = iZoneRpm;
        this.iMaxPower = iMaxPower;
        this.derivativeFilterAlpha = derivativeFilterAlpha;
        this.nominalVolts = nominalVolts;
    }

    /**
     * Run one step of the controller. Call this once per robot loop.
     *
     * @param targetRpm speed we want. Zero or less means "off" and returns zero power
     * @param measuredRpm speed we have
     * @param dtSeconds time since the previous call
     * @param batteryVolts present battery voltage. Power is a fraction of the battery voltage, so a
     *                     tired battery needs a larger fraction to deliver the same motor voltage
     * @return motor power, between 0 and 1
     */
    public double update(double targetRpm, double measuredRpm, double dtSeconds, double batteryVolts) {
        if (targetRpm <= 0) {
            reset();
            feedforwardTerm = proportionalTerm = derivativeTerm = output = 0;
            return 0;
        }

        // A first call, or a long pause in the loop, gives no usable time step. Skip I and D then.
        boolean validDt = dtSeconds > 0 && dtSeconds < 0.5;
        double error = targetRpm - measuredRpm;

        feedforwardTerm = kS + kV * targetRpm;
        proportionalTerm = kP * error;

        if (validDt && hasLastMeasurement) {
            double rawRpmPerSec = (measuredRpm - lastMeasuredRpm) / dtSeconds;
            filteredRpmPerSec += derivativeFilterAlpha * (rawRpmPerSec - filteredRpmPerSec);
        }
        derivativeTerm = -kD * filteredRpmPerSec;
        lastMeasuredRpm = measuredRpm;
        hasLastMeasurement = true;

        // Scale everything for the battery: the same power gives less motor voltage on a low battery.
        double voltageScale = (batteryVolts > 1) ? nominalVolts / batteryVolts : 1.0;
        double beforeIntegral = (feedforwardTerm + proportionalTerm + derivativeTerm) * voltageScale;
        double previousOutput = beforeIntegral + integral * voltageScale;

        // Anti-windup: integrate only near the target, and never when already pushing against a limit.
        boolean pushingIntoLimit = (previousOutput >= maxOutput && error > 0)
                || (previousOutput <= minOutput && error < 0);
        if (validDt && Math.abs(error) < iZoneRpm && !pushingIntoLimit) {
            integral += kI * error * dtSeconds;
        }
        integral = clamp(integral, -iMaxPower, iMaxPower);

        output = clamp(beforeIntegral + integral * voltageScale, minOutput, maxOutput);
        return output;
    }

    /** Forget all history. Call this when the shooter is switched off. */
    public void reset() {
        integral = 0;
        filteredRpmPerSec = 0;
        hasLastMeasurement = false;
    }

    private static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(high, value));
    }

    // These getters let us graph each term in Panels and see who is doing the work.
    public double getFeedforwardTerm() { return feedforwardTerm; }
    public double getProportionalTerm() { return proportionalTerm; }
    public double getIntegralTerm() { return integral; }
    public double getDerivativeTerm() { return derivativeTerm; }
    public double getOutput() { return output; }
}
