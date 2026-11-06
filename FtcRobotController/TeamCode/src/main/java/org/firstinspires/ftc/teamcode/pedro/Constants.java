package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
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
    public static boolean foresightTuned = false;

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

    // AutoTune OctoQuad Tuner result, second run (6 Nov 2026): goBILDA four-bar pods on ports 0
    // and 1. The first run was half a turn short, which also flipped its y pod direction.
    public static OctoQuadConfig localizerConfig = new OctoQuadConfig(c -> {
        c.name.set("octoquad");
        c.xPodPort.set(0);
        c.yPodPort.set(1);
        c.ticksPerUnit.set(505.316944406);
        c.xPodOffset.set(6.594488188976379);
        c.yPodOffset.set(-5.039370078740157);
        c.xPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
        c.yPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
        c.headingScalar.set(1.0518035195987139);
    });

    // Library defaults until the Foresight Tuner runs. Manual driving works without it;
    // following paths and holding position need it.
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> { });

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
