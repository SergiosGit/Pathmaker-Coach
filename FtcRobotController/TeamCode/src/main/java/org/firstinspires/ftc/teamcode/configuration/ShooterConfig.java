package org.firstinspires.ftc.teamcode.configuration;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Every shooter number you may want to change, all in one place.
 *
 * Because of @Configurable these appear in the Panels "Configurables" tab and can be changed while
 * the robot runs, with no rebuild. Changes made in Panels are LOST when the app restarts, so once
 * you find a good value, type it in here as well.
 *
 * Gains marked "start" come from the simulation in docs/shooter_sim. They are starting points:
 * tune them on the real robot, following docs/SHOOTER-PID.md.
 */
@Configurable
public class ShooterConfig {

    // ---- Speeds (RPM of the Rhino wheel). These replace the old 0.90 / 0.95 / 1.00 powers ----
    // Placeholders: find the real values by test shooting. Stay at or below about 5000 RPM if you
    // want rapid fire, because the motor has little spare torque left above that speed.
    public static double presetLowRpm = 4000;   // gamepad2 X
    public static double presetMidRpm = 4500;   // gamepad2 Y
    public static double presetHighRpm = 5000;  // gamepad2 B
    public static double autoTargetRpm = 4500;  // used by the autonomous paths in PathDetails
    public static double dpadStepRpm = 100;     // gamepad2 dpad up / down
    public static double reversePower = -0.25;  // gamepad2 A while held, to clear a jam

    // ---- Feedforward: the power we calculate in advance ----
    public static double kS = 0.03;             // start: friction, from the motor's 0.25 A free current
    public static double kV = 1.0 / 6000;       // start: 12 V gives about 6000 RPM with no load

    // ---- PID: the correction for whatever feedforward got wrong (power per RPM of error) ----
    public static double kP = 0.002;            // start: a 100 RPM error adds 0.2 power
    public static double kI = 0.0006;           // start: a 100 RPM error adds 0.06 power each second
    public static double kD = 0;                // leave at 0 unless you are experimenting

    // ---- Controller limits ----
    public static double iZoneRpm = 300;        // only integrate when closer than this to the target
    public static double iMaxPower = 0.15;      // the most power the integral may contribute
    public static double derivativeFilterAlpha = 0.3; // 0 to 1, smaller is smoother but slower
    public static double nominalVolts = 12.0;   // battery voltage that kS and kV refer to

    // ---- "Ready to shoot" ----
    public static double atSpeedToleranceRpm = 100; // how close to target counts as "ready"
    public static double atSpeedHoldMs = 60;        // must stay in tolerance this long to count
    public static double shotDipRpm = 150;          // a sudden drop this big is counted as a shot
    public static boolean gateIntakeOnSpeed = false; // true: teleop feeds only when at speed. See the doc

    // ---- Hardware ----
    public static String octoQuadName = "octoquad"; // name in the robot configuration
    public static int encoderChannel = 0;           // OctoQuad channel, 0 to 7
    public static boolean reverseMotor = false;     // flip if the wheel spins the wrong way
    public static boolean reverseEncoder = false;   // flip if measured RPM is negative
    public static int velocityWindowMs = 20;        // OctoQuad counts encoder steps for this long.
                                                    // Shooter limits it so the 16-bit register cannot wrap

    // ---- Safety ----
    public static double encoderFaultMs = 1000;     // power on but no speed for this long = fault
    public static double reverseMaxRpm = 500;       // never reverse the motor when faster than this
    public static double maxPlausibleRpm = 7500;    // faster than the motor can spin = a bad reading

    // ---- Used only by the "Shooter Tuning" OpMode ----
    public static double tuningTargetRpm = 4500;
    public static double tuningOpenLoopPower = 0.7; // power for measuring kV
    public static double tuningFeedPower = 0.7;     // intake power while holding left trigger
    public static int tuningExtraLoopDelayMs = 25;  // imitates the 20-40 ms loop of the full robot

    /**
     * REV Through Bore Encoder (REV-11-3174), which must turn at the same speed as the Rhino wheel.
     * Not tunable: it is a property of the encoder. To fall back to the motor's built-in encoder,
     * change this to 28 and velocityWindowMs to 40.
     */
    public static final double ENCODER_COUNTS_PER_REV = 8192;
}
