package org.firstinspires.ftc.teamcode.babybot.op.test;


import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp (name = "Localizing Clump", group = "test")
@Configurable
public class LocalizingClump extends LinearOpMode {

    public static double pollenDiameter = 2.8;
    public static double nectarDiameter = 3.6;
    public static double camUpOffset = 5.625;
    public static double camRightOffset = 0;
    public static double camForwardOffset = 0;
    public static double camPitchUpOffset = 0;
    public static double camYawRightOffset = 0;
    public static boolean isRed = true;
    public static double switchTime = 200;


    private static Limelight3A limelight;
    private static ElapsedTime switchTimer = new ElapsedTime();
    private static double ballDiameter = 2.8;
    private static Pose2D pollenPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    private static Pose2D nectarPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    private static Pose2D badNectarPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    private Follower follower;

    public void runOpMode() throws InterruptedException {

        Pose startPose = PoseFactory.degrees().of(0, 0, 0);
        Scheduler.reset();
        Telemetry tele = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        waitForStart();

        limelight.start();
        limelight.pipelineSwitch(0);

        while(opModeIsActive()) {

            follower.update();
            Scheduler.execute();

            if(switchTimer.milliseconds() > switchTime) {
                switchTimer.reset();
                setLimelightPipeline();
            }

            LLResult result = limelight.getLatestResult();

            if(result != null && result.isValid()) {
                getBallPose(result);
                if(gamepad1.aWasPressed()) driveToPollen(pollenPose);
                updateTele(tele, result);
            }
        }
    }

    public void driveToPollen(Pose2D pose) {
        Pose currentPose = PoseFactory.degrees().of(
                follower.pose().x(),
                follower.pose().y(),
                follower.pose().heading());
        Pose targetPose = PoseFactory.degrees().of(
                follower.pose().x() + pose.getX(DistanceUnit.INCH),
                follower.pose().y() + pose.getY(DistanceUnit.INCH),
                follower.pose().heading()
                );

        Path path = Paths.line(currentPose, targetPose);

        Scheduler.schedule(follow(follower, path));
    }

    public static void setLimelightPipeline() {
        int currentPipe = limelight.getStatus().getPipelineIndex();

        if(currentPipe == 2) ballDiameter = pollenDiameter;
        else ballDiameter = nectarDiameter;

        currentPipe++;
        currentPipe %= 3;

        limelight.pipelineSwitch(currentPipe);
    }

    public static void getBallPose(LLResult result) {
        int pipe = limelight.getStatus().getPipelineIndex();

        double yaw = Math.toRadians(result.getTx() + camYawRightOffset);
        double pitch = Math.toRadians(camPitchUpOffset + result.getTy());

        double t = -(camUpOffset - ballDiameter / 2) / Math.sin(pitch);
        double forwardDist = t * Math.cos(yaw) * Math.cos(pitch) + camForwardOffset;
        double rightDist = t * Math.sin(yaw) * Math.cos(pitch) + camRightOffset;

        Pose2D ballPose = new Pose2D(DistanceUnit.INCH, forwardDist, -rightDist, AngleUnit.DEGREES, 0);

        if(pipe == 0) pollenPose = ballPose;
        else if((pipe == 1) == isRed) {
            nectarPose = ballPose;
        } else {
           badNectarPose = ballPose;
        }
    }

    private void updateTele(Telemetry tele, LLResult result) {

        tele.addLine(String.format("Pollen Pose f/s %6.1f %6.1f",
                pollenPose.getY(DistanceUnit.INCH),
                pollenPose.getX(DistanceUnit.INCH)));
        tele.addLine(String.format("Nectar Pose f/s %6.1f %6.1f",
                nectarPose.getY(DistanceUnit.INCH),
                nectarPose.getX(DistanceUnit.INCH)));
        tele.addLine(String.format("Bad nectar Pose f/s %6.1f %6.1f",
                badNectarPose.getY(DistanceUnit.INCH),
                badNectarPose.getX(DistanceUnit.INCH)));
        tele.addData("pipeline index", result.getPipelineIndex());
        tele.addData("ll running", limelight.isRunning());

        tele.update();
    }
}
