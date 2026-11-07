package org.firstinspires.ftc.teamcode.pedro;

import static org.firstinspires.ftc.teamcode.calibration.Calibration.getDouble;
import static org.firstinspires.ftc.teamcode.calibration.Calibration.getEnum;
import static org.firstinspires.ftc.teamcode.calibration.Calibration.getInt;
import static org.firstinspires.ftc.teamcode.calibration.Calibration.getString;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.calibration.Calibration;

/**
 * Pedro Pathing 3 setup for the bay-b bot. All values come from the calibration
 * (http://192.168.43.1:8090 and calibration/RobotCalibration.java); nothing is set here.
 * Each method builds a fresh config, so a value measured a moment ago is used at the next OpMode init.
 */
public class Constants {

    private Constants() { }

    public static MecanumConfig drivetrainConfig() {
        return new MecanumConfig(c -> {
            c.frontLeftName.set(getString("mecanum.frontLeftName"));
            c.frontRightName.set(getString("mecanum.frontRightName"));
            c.backLeftName.set(getString("mecanum.backLeftName"));
            c.backRightName.set(getString("mecanum.backRightName"));
            c.frontLeftDirection.set(getEnum("mecanum.frontLeftDirection", DcMotorSimple.Direction.class));
            c.frontRightDirection.set(getEnum("mecanum.frontRightDirection", DcMotorSimple.Direction.class));
            c.backLeftDirection.set(getEnum("mecanum.backLeftDirection", DcMotorSimple.Direction.class));
            c.backRightDirection.set(getEnum("mecanum.backRightDirection", DcMotorSimple.Direction.class));
        });
    }

    public static OctoQuadConfig localizerConfig() {
        return new OctoQuadConfig(c -> {
            c.name.set(getString("octoquad.name"));
            c.xPodPort.set(getInt("octoquad.xPodPort"));
            c.yPodPort.set(getInt("octoquad.yPodPort"));
            c.ticksPerUnit.set(getDouble("octoquad.ticksPerUnit"));
            c.xPodOffset.set(getDouble("octoquad.xPodOffset"));
            c.yPodOffset.set(getDouble("octoquad.yPodOffset"));
            c.xPodDirection.set(getEnum("octoquad.xPodDirection", OctoQuad.EncoderDirection.class));
            c.yPodDirection.set(getEnum("octoquad.yPodDirection", OctoQuad.EncoderDirection.class));
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.offsetUnits.set(DistanceUnit.INCH);
            c.i2cRecoveryMode.set(getEnum("octoquad.i2cRecoveryMode", OctoQuad.I2cRecoveryMode.class));
            c.headingScalar.set(getDouble("octoquad.headingScalar"));
        });
    }

    /** Defaults have zero gains: Pedro can't drive a path or hold position until this is calibrated. */
    public static ForesightConfig foresightConfig() {
        return new ForesightConfig(c -> {
            Controller primaryForward = Controller.proportional(getDouble("foresight.forwardTranslational Primary kP"));
            Controller secondaryForward = Controller.proportional(getDouble("foresight.forwardTranslational Secondary kP"));
            Controller primaryLateral = Controller.proportional(getDouble("foresight.strafeTranslational Primary kP"));
            Controller secondaryLateral = Controller.proportional(getDouble("foresight.strafeTranslational Secondary kP"));

            c.forwardTranslational.set(Controller.piecewise(secondaryForward).put(2.5, primaryForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryLateral).put(2.5, primaryLateral));

            c.coast.set(Controller.proportionalFeedforward(getDouble("foresight.coast kV")));
            c.brake.set(Controller.proportionalFeedforward(getDouble("foresight.brake kV")));

            c.headingFeedback.set(Controller.proportional(getDouble("foresight.heading kP")));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(
                    getDouble("foresight.headingBrakingLinearCoefficient"),
                    getDouble("foresight.headingBrakingQuadraticCoefficient")));

            c.linearBrakeCoefficients.set(Matrix.diag(
                    getDouble("foresight.forwardBrakingLinearCoefficient"),
                    getDouble("foresight.strafeBrakingLinearCoefficient")));
            c.quadraticBrakeCoefficients.set(Matrix.diag(
                    getDouble("foresight.forwardBrakingQuadraticCoefficient"),
                    getDouble("foresight.strafeBrakingQuadraticCoefficient")));

            c.maxAchievableForwardVelocity.set(getDouble("foresight.maxAchievableForwardVelocity"));
            c.maxAchievableStrafeVelocity.set(getDouble("foresight.maxAchievableStrafeVelocity"));
            c.naturalForwardDeceleration.set(getDouble("foresight.naturalForwardDeceleration"));
            c.naturalStrafeDeceleration.set(getDouble("foresight.naturalStrafeDeceleration"));
        });
    }

    /** True once Pedro's Foresight values are committed or measured, not defaults. */
    public static boolean foresightTuned() {
        return Calibration.groupCalibrated("Pedro Foresight");
    }

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new OctoQuadLocalizer(hardwareMap, localizerConfig()),
                new Mecanum(hardwareMap, drivetrainConfig()),
                new Foresight(foresightConfig()));
    }
}
