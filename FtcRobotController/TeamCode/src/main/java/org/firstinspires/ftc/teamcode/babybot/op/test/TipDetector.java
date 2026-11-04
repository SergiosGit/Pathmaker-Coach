package org.firstinspires.ftc.teamcode.babybot.op.test;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

// only works while stationary with normal webcam, works while moving with 240 fps camera
@Autonomous (name = "Tip detector", group = "test")
public class TipDetector extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean isTipping = false;
    private int emptyCount = 0;

    public void runOpMode() throws InterruptedException {

        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(640, 360))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();

        waitForStart();

        while(opModeIsActive()) {


            List<AprilTagDetection> detections = aprilTag.getDetections();
            if(detections.isEmpty()) emptyCount++;
            else emptyCount = 0;

            isTipping = emptyCount >= 20;

            telemetry.addData("num tags in view", detections.size());
            telemetry.addData("is tipping", isTipping);

            telemetry.update();
        }

        visionPortal.close();
    }
}
