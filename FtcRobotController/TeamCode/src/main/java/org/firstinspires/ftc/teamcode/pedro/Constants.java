package org.firstinspires.ftc.teamcode.pedro;

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

/**
 * Pedro Pathing 3 setup for the bay-b bot.
 *
 * Each config block below is replaced by the code that AutoTune (http://192.168.43.1:10158)
 * prints at the end of a procedure. Run them in this order, paste each result over the
 * matching block, set its *Tuned flag to true, and redeploy before the next procedure:
 *   1. Mecanum Tuner   -> drivetrainConfig
 *   2. OctoQuad Tuner  -> localizerConfig
 *   3. Foresight Tuner -> foresightConfig (needs 1 and 2, so it is enabled in Tuning.java)
 */
public class Constants {

    // Set each flag to true after pasting that block's AutoTune result.
    public static boolean drivetrainTuned = true;
    public static boolean localizerTuned = true;
    public static boolean foresightTuned = true;

    // AutoTune Mecanum Tuner result (6 Nov 2026).
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("front_left");
        c.frontRightName.set("front_right");
        c.backLeftName.set("back_left");
        c.backRightName.set("back_right");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // AutoTune OctoQuad Tuner result, third run (6 Nov 2026): goBILDA four-bar pods. The forward
    // pod is plugged into port 1 and the strafe pod into port 0. Earlier runs assumed the
    // opposite and measured the wrong pod in each step.
    public static OctoQuadConfig localizerConfig = new OctoQuadConfig(c -> {
        c.name.set("octoquad");
        c.xPodPort.set(1);
        c.yPodPort.set(0);
        c.ticksPerUnit.set(505.316944406);
        c.xPodOffset.set(4.468503937007874);
        c.yPodOffset.set(6.850393700787401);
        c.xPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
        c.yPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
        c.headingScalar.set(1.0521839720139383);
    });

    // AutoTune Foresight Tuner result (6 Nov 2026).
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryTranslationalForward = Controller.proportional(0.1912510589131322);
        Controller secondaryTranslationalForward = Controller.proportional(0.07066215594599003);
        Controller primaryTranslationalLateral = Controller.proportional(0.22345115371010554);
        Controller secondaryTranslationalLateral = Controller.proportional(0.08255923057109245);

        c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

        c.coast.set(Controller.proportionalFeedforward(0.016068837025916338));
        c.brake.set(Controller.proportionalFeedforward(0.013658511472028886));

        c.headingFeedback.set(Controller.proportional(2.427462770182534));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.028103842539870735, 0.005619561123838429));

        c.linearBrakeCoefficients.set(Matrix.diag(0.030382513987097765, 0.04831570754977478));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.002088581762850105, 0.0016405745909135573));

        c.maxAchievableForwardVelocity.set(62.05122551694287);
        c.maxAchievableStrafeVelocity.set(47.99371093640487);
        c.naturalForwardDeceleration.set(30.74715869191283);
        c.naturalStrafeDeceleration.set(56.511232824202615);
    });

    public static Follower create(HardwareMap hardwareMap) {
        if (!drivetrainTuned || !localizerTuned) {
            throw new IllegalStateException("Pedro is not tuned yet. Run the Mecanum and OctoQuad "
                    + "procedures in AutoTune (http://192.168.43.1:10158), paste the results into "
                    + "pedro/Constants.java and set drivetrainTuned and localizerTuned to true.");
        }
        return new Follower(
                new OctoQuadLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig));
    }
}
