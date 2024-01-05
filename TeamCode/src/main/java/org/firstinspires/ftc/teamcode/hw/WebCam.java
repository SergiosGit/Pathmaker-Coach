package org.firstinspires.ftc.teamcode.hw;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.op.RobotPose;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class WebCam {
    private static LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    private static VisionPortal visionPortal;

    public static List<AprilTagDetection> currentDetections = null;
    public static double distanceToTarget = 0, offsetToTarget = 0, angleToTarget = 0;
    public static double fieldForwardPosition = 0, fieldStrafePosition = 0, fieldHeadingAngle = 0;


    public static void init(LinearOpMode opMode, Telemetry telemetry) throws InterruptedException {
        myOpMode = opMode;
        initAprilTag();
        while (WebCam.setManualExposure(6, 250, telemetry) == null) {
            Thread.sleep(100);
        }
    }
    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private static AprilTagProcessor aprilTag;
    private static void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }   // end method initAprilTag()

    /*
         Manually set the camera gain and exposure.
         This can only be called AFTER calling initAprilTag(), and only works for Webcams;
        */
    public static Object setManualExposure(int exposureMS, int gain, Telemetry telemetry) throws InterruptedException {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            telemetry.addData("Error", "visionPortal is null");
            telemetry.update();
            return null;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Error", "Camera is not streaming");
            telemetry.update();
            return null;
        }

        // Set camera controls\
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            Thread.sleep(50);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        Thread.sleep(20);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        Thread.sleep(20);
        telemetry.addData("Camera Exposure initialized (ms)", exposureControl.getExposure(TimeUnit.MILLISECONDS));
        telemetry.update();
        return 1;
    }

    public static boolean detectionAprilTag(int ID) {
        currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == ID) {
                    distanceToTarget = detection.ftcPose.range;
                    angleToTarget = -detection.ftcPose.yaw; // when robot is looking directly at target (bearing = 0)
                    offsetToTarget = detection.ftcPose.x;
                    fieldForwardPosition = RobotPose.getForward_in();
                    fieldStrafePosition = RobotPose.getStrafe_in();
                    fieldHeadingAngle = RobotPose.getHeadingAngle_deg();
                    return true;
                }
            }
        }
        return false;
    }

    public static void telemetryAprilTag(Telemetry telemetry) {
        boolean found = detectionAprilTag(583);
        telemetry.addData("AprilTag found", found);
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

}
