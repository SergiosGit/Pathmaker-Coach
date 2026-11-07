package org.firstinspires.ftc.teamcode.calibration;

import static org.firstinspires.ftc.teamcode.calibration.Calibration.define;

/**
 * Every calibrated value, its group on the web page, and its default. Defaults are chosen to be
 * safe on an untuned robot: Pedro's controller gains are zero, so Pedro cannot drive a path or
 * hold a position until its group is calibrated (the bay-b bot's Drive test then drives by hand).
 *
 * The ids of Pedro's values match the names its tuning procedures report, so a procedure's
 * results land here without translation.
 */
final class CalibrationKeys {

    static final String ROBOT = "Robot";
    static final String DRIVETRAIN = "Drivetrain motors";
    static final String ODOMETRY = "Odometry (OctoQuad)";
    static final String FORESIGHT = "Pedro Foresight";
    static final String PATHMAKER = "PathMaker drive";

    private CalibrationKeys() { }

    static void defineAll() {
        define(ROBOT, "robot.motorRpm", "Drive motor free speed", "rpm", 312.0,
                "From the motor's data sheet. Only used to estimate defaults.");
        define(ROBOT, "robot.wheelDiameterIn", "Drive wheel diameter", "in", 4.09,
                "goBILDA 104 mm mecanum = 4.09 in. Only used to estimate defaults.");

        define(DRIVETRAIN, "mecanum.frontLeftName", "Front left motor name", "", "front_left", "Name in the robot configuration");
        define(DRIVETRAIN, "mecanum.frontRightName", "Front right motor name", "", "front_right", "Name in the robot configuration");
        define(DRIVETRAIN, "mecanum.backLeftName", "Back left motor name", "", "back_left", "Name in the robot configuration");
        define(DRIVETRAIN, "mecanum.backRightName", "Back right motor name", "", "back_right", "Name in the robot configuration");
        define(DRIVETRAIN, "mecanum.frontLeftDirection", "Front left direction", "", "REVERSE", "FORWARD or REVERSE");
        define(DRIVETRAIN, "mecanum.frontRightDirection", "Front right direction", "", "FORWARD", "FORWARD or REVERSE");
        define(DRIVETRAIN, "mecanum.backLeftDirection", "Back left direction", "", "REVERSE", "FORWARD or REVERSE");
        define(DRIVETRAIN, "mecanum.backRightDirection", "Back right direction", "", "FORWARD", "FORWARD or REVERSE");

        define(ODOMETRY, "octoquad.name", "OctoQuad name", "", "octoquad", "Name in the robot configuration");
        define(ODOMETRY, "octoquad.xPodPort", "Forward pod port", "", 0, "OctoQuad port of the pod that rolls when the robot drives forward");
        define(ODOMETRY, "octoquad.yPodPort", "Strafe pod port", "", 1, "OctoQuad port of the pod that rolls when the robot strafes");
        define(ODOMETRY, "octoquad.xPodDirection", "Forward pod direction", "", "FORWARD", "FORWARD or REVERSE: forward motion must count up");
        define(ODOMETRY, "octoquad.yPodDirection", "Strafe pod direction", "", "FORWARD", "FORWARD or REVERSE: motion to the left must count up");
        define(ODOMETRY, "octoquad.ticksPerUnit", "Pod ticks per inch", "ticks/in", 505.316944406,
                "goBILDA four-bar pod = 505.3, swing-arm pod = 336.9");
        define(ODOMETRY, "octoquad.xPodOffset", "Forward pod offset", "in", 0.0, "Measured by the OctoQuad step");
        define(ODOMETRY, "octoquad.yPodOffset", "Strafe pod offset", "in", 0.0, "Measured by the OctoQuad step");
        define(ODOMETRY, "octoquad.headingScalar", "Heading scale", "", 1.0, 0.005,
                "Corrects the OctoQuad gyro. Two runs must agree within 0.5%");
        define(ODOMETRY, "octoquad.i2cRecoveryMode", "I2C recovery mode", "", "MODE_1_PERIPH_RST_ON_FRAME_ERR", "Leave as is");

        for (String id : new String[]{"mecanum.frontLeftDirection", "mecanum.frontRightDirection",
                "mecanum.backLeftDirection", "mecanum.backRightDirection",
                "octoquad.xPodDirection", "octoquad.yPodDirection"}) {
            Calibration.choices(id, "FORWARD", "REVERSE");
        }
        Calibration.choices("octoquad.i2cRecoveryMode", "NONE", "MODE_1_PERIPH_RST_ON_FRAME_ERR",
                "MODE_2_M1_PLUS_SCL_IDLE_ONESHOT_TGL");

        define(FORESIGHT, "foresight.maxAchievableForwardVelocity", "Top speed forward", "in/s", 55.0, "Full power, measured");
        define(FORESIGHT, "foresight.maxAchievableStrafeVelocity", "Top speed strafing", "in/s", 45.0, "Full power, measured");
        define(FORESIGHT, "foresight.naturalForwardDeceleration", "Coasting deceleration forward", "in/s²", 40.0, "Power off, measured");
        define(FORESIGHT, "foresight.naturalStrafeDeceleration", "Coasting deceleration strafing", "in/s²", 50.0, "Power off, measured");
        define(FORESIGHT, "foresight.headingBrakingLinearCoefficient", "Heading braking, linear", "", 0.0, "Turn stopping model");
        define(FORESIGHT, "foresight.headingBrakingQuadraticCoefficient", "Heading braking, quadratic", "", 0.0, "Turn stopping model");
        define(FORESIGHT, "foresight.heading kP", "Heading kP", "", 0.0, "Heading controller gain");
        define(FORESIGHT, "foresight.forwardBrakingLinearCoefficient", "Forward braking, linear", "", 0.0, "Stopping distance model");
        define(FORESIGHT, "foresight.forwardBrakingQuadraticCoefficient", "Forward braking, quadratic", "", 0.0, "Stopping distance model");
        define(FORESIGHT, "foresight.strafeBrakingLinearCoefficient", "Strafe braking, linear", "", 0.0, "Stopping distance model");
        define(FORESIGHT, "foresight.strafeBrakingQuadraticCoefficient", "Strafe braking, quadratic", "", 0.0, "Stopping distance model");
        define(FORESIGHT, "foresight.forwardTranslational Primary kP", "Forward kP, far from path", "", 0.0, "Used above 2.5 in of error");
        define(FORESIGHT, "foresight.forwardTranslational Secondary kP", "Forward kP, near path", "", 0.0, "Used below 2.5 in of error");
        define(FORESIGHT, "foresight.strafeTranslational Primary kP", "Strafe kP, far from path", "", 0.0, "Used above 2.5 in of error");
        define(FORESIGHT, "foresight.strafeTranslational Secondary kP", "Strafe kP, near path", "", 0.0, "Used below 2.5 in of error");
        define(FORESIGHT, "foresight.coast kV", "Coast kV", "", 0.0, "Speed feedback while coasting");
        define(FORESIGHT, "foresight.brake kV", "Brake kV", "", 0.0, "Speed feedback while braking");

        define(PATHMAKER, "pathmaker.kSForward", "Start-up power forward (kS)", "power", 0.08, "Smallest power that starts the robot moving");
        define(PATHMAKER, "pathmaker.kSStrafe", "Start-up power strafing (kS)", "power", 0.12, "Smallest power that starts the robot moving");
        define(PATHMAKER, "pathmaker.kSTurn", "Start-up power turning (kS)", "power", 0.08, "Smallest power that starts the robot turning");
        define(PATHMAKER, "pathmaker.brakeDecelForward", "Braking deceleration forward", "in/s²", 80.0, "Power off, motors braking");
        define(PATHMAKER, "pathmaker.brakeDecelStrafe", "Braking deceleration strafing", "in/s²", 100.0, "Power off, motors braking");
        define(PATHMAKER, "pathmaker.maxTurnSpeed", "Top turning speed", "deg/s", 300.0, "Full power, measured");
        define(PATHMAKER, "pathmaker.brakeDecelTurn", "Braking deceleration turning", "deg/s²", 600.0, "Power off, motors braking");
        define(PATHMAKER, "pathmaker.stillNoiseXY", "Speed noise standing still", "in/s", 0.3, "Below this the robot counts as stopped");
        define(PATHMAKER, "pathmaker.stillNoiseTurn", "Turn speed noise standing still", "deg/s", 1.0, "Below this the robot counts as stopped");
    }
}
