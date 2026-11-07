package org.firstinspires.ftc.teamcode.calibration;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;

import org.firstinspires.ftc.teamcode.calibration.runner.CalibrationSession;
import org.firstinspires.ftc.teamcode.calibration.runner.CalibrationSession.Step;
import org.firstinspires.ftc.teamcode.calibration.steps.AxesCheck;
import org.firstinspires.ftc.teamcode.calibration.steps.DriveCharacterization;
import org.firstinspires.ftc.teamcode.calibration.steps.ForesightTuner;
import org.firstinspires.ftc.teamcode.calibration.steps.MecanumTuner;
import org.firstinspires.ftc.teamcode.calibration.steps.OctoQuadTuner;
import org.firstinspires.ftc.teamcode.calibration.steps.Tests;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.Arrays;

/**
 * The steps on the calibration page, in the order to run them. A step's id is also the prefix of
 * the values it saves (MecanumTuner's "frontLeftName" → "mecanum.frontLeftName").
 */
public final class CalibrationSteps {

    private CalibrationSteps() { }

    public static void install() {
        CalibrationSession.get().setSteps(Arrays.asList(
                new Step("mecanum", CalibrationKeys.DRIVETRAIN, "Motor directions",
                        "Spins each wheel in turn; you say which way it turned. About 1 minute.",
                        keys(CalibrationKeys.DRIVETRAIN), MecanumTuner::new),
                new Step("axes", CalibrationKeys.ODOMETRY, "Odometry ports and directions",
                        "Push the robot forward, then left. Finds swapped pod cables. Nothing is powered.",
                        new String[]{"octoquad.xPodPort", "octoquad.yPodPort", "octoquad.xPodDirection", "octoquad.yPodDirection"},
                        AxesCheck::new),
                new Step("octoquad", CalibrationKeys.ODOMETRY, "OctoQuad heading and offsets",
                        "Turn the robot by hand 10 times, then push it; the robot then spins to find the pod offsets. "
                                + "Run twice: the heading scale counts once two runs agree.",
                        new String[]{"octoquad.headingScalar", "octoquad.xPodOffset", "octoquad.yPodOffset", "octoquad.ticksPerUnit"},
                        OctoQuadTuner::new),
                new Step("foresight", CalibrationKeys.FORESIGHT, "Pedro Foresight",
                        "12 powered runs: top speed, coasting, braking, heading and position gains. Needs about 8 ft of floor.",
                        keys(CalibrationKeys.FORESIGHT),
                        () -> new ForesightTuner(
                                hw -> new OctoQuadLocalizer(hw, Constants.localizerConfig()),
                                hw -> new Mecanum(hw, Constants.drivetrainConfig()))),
                new Step("pathmaker", CalibrationKeys.PATHMAKER, "PathMaker drive",
                        "Start-up power, braking and turning speed, with distance and time limits. About 4 ft of floor.",
                        keys(CalibrationKeys.PATHMAKER), DriveCharacterization::new),
                new Step("tests", "Checks", "Pedro tests",
                        "Drive Pedro along a line, a curve or hold a position, to check the calibration.",
                        new String[0],
                        () -> new Tests(
                                hw -> new Mecanum(hw, Constants.drivetrainConfig()),
                                hw -> new OctoQuadLocalizer(hw, Constants.localizerConfig()),
                                () -> new Foresight(Constants.foresightConfig())))));
    }

    private static String[] keys(String group) {
        java.util.List<String> out = new java.util.ArrayList<>();
        for (Calibration.Key k : Calibration.keys()) if (k.group.equals(group)) out.add(k.id);
        return out.toArray(new String[0]);
    }
}
