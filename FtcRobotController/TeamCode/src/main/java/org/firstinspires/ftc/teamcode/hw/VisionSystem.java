package org.firstinspires.ftc.teamcode.hw;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static java.lang.Thread.sleep;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.configuration.GameConfig;
import org.firstinspires.ftc.teamcode.configuration.RobotConfig;
import org.firstinspires.ftc.teamcode.op.RobotPose;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class VisionSystem {

    private static WebcamName webcam1, webcam2;
    private static CameraName switchableCamera;
    private static VisionPortal visionPortal;
    private static AprilTagProcessor aprilTag;
    private static boolean multipleCameras;

    private static int resolutionWidth = RobotConfig.Camera.getResolutionWidth();
    private static int resolutionHeight = RobotConfig.Camera.getResolutionHeight();
    private static VisionPortal.StreamFormat streamFormat = RobotConfig.Camera.getStreamFormat();

    private static boolean enableLivePreview = RobotConfig.Camera.enableLivePreview();

    private static double fx = RobotConfig.Camera.getFx();
    private static double fy = RobotConfig.Camera.getFy();
    private static double cx = RobotConfig.Camera.getCx();
    private static double cy = RobotConfig.Camera.getCy();
    private static Position cameraPosition = RobotConfig.Camera.getCameraPosition();
    private static YawPitchRollAngles cameraOrientation = RobotConfig.Camera.getCameraOrientation();
    private static AprilTagLibrary library = GameConfig.AprilTags.getAprilTagLibrary();

    private static boolean customExposure = RobotConfig.Camera.useCustomExposure();
    private static int exposure = RobotConfig.Camera.getExposure();
    private static int gain = RobotConfig.Camera.getGain();



    public static void createVisionSystem(WebcamName webcam1, WebcamName webcam2) throws  InterruptedException {
        VisionSystem.webcam1 = webcam1;
        VisionSystem.webcam2 = webcam2;
        multipleCameras = true;
        initVisionSystem();
        if(customExposure) { setCustomExposure(); }
    }

    public static void createVisionSystem(WebcamName webcamOne) throws  InterruptedException {
        webcam1 = webcamOne;
        multipleCameras = false;
        initVisionSystem();
        if(RobotConfig.Camera.useCustomExposure()) { setCustomExposure(); }
    }

    public static VisionPortal getVisionPortal() {
        return visionPortal;
    }

    public static AprilTagProcessor getAprilTagProcessor() {
        return aprilTag;
    }

    public static void stopStreaming() {
        if(isStreaming()) {
            visionPortal.stopStreaming();
        }
    }

    public static void startStreaming() {
        if(!isStreaming()) {
            visionPortal.resumeStreaming();
        }
    }

    public static void closePortal() {
        visionPortal.close();
    }

    public static void enableAprilTagProcessing() {
        visionPortal.setProcessorEnabled(aprilTag, true);
    }

    public static void disableAprilTagProcessing() {
        visionPortal.setProcessorEnabled(aprilTag, false);
    }

    public static VisionPortal.CameraState getStreamingState() {
        return visionPortal.getCameraState();
    }

    public static boolean isStreaming() {
        return getStreamingState() == VisionPortal.CameraState.STREAMING;
    }

    public static boolean switchCameras() {

        if(!multipleCameras) { return false; }

        if(visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.setActiveCamera(visionPortal.getActiveCamera().equals(webcam1) ? webcam2: webcam1);
            return true;
        }
        return false;
    }

    public static WebcamName getCurrentWebcam() {
        return visionPortal.getActiveCamera();
    }

    private static void initVisionSystem() {

        AprilTagProcessor.Builder tagBuilder = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(library);

        if(RobotConfig.Camera.useCustomCameraCalibration) {
            tagBuilder.setLensIntrinsics(fx, fy, cx, cy);
        }

        aprilTag = tagBuilder.build();

        if(multipleCameras) {
            switchableCamera = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(webcam1, webcam2);
        }

        VisionPortal.Builder portalBuilder = new VisionPortal.Builder()
                .setCamera(multipleCameras? switchableCamera: webcam1)
                .setCameraResolution(new Size(resolutionWidth, resolutionHeight))
                .setStreamFormat(streamFormat)
                .enableLiveView(enableLivePreview)
                .addProcessor(aprilTag);

        visionPortal = portalBuilder.build();
    }

    // meant to be run only at init
    private static boolean setCustomExposure() throws InterruptedException{

        ElapsedTime timeSinceStreamStart = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while(timeSinceStreamStart.time() <= 2000 && !isStreaming()) {
            sleep(50);
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

        exposureControl.setMode(ExposureControl.Mode.Manual);

        timeSinceStreamStart.reset();
        while(!isStreaming()) {
            visionPortal.resumeStreaming();
            if(timeSinceStreamStart.time() > 2000) { return false; }
        }

        exposureControl.setExposure(exposure, TimeUnit.MILLISECONDS);
        gainControl.setGain(gain);

        return true;
    }



    public static class AprilTag {

        public static List<AprilTagDetection> getAprilTagDetections() {
            return aprilTag.getDetections();
        }

        // SDK 12.0: a detection is either a single tag (has an ID) or a cluster of tags (has a name,
        // no ID). This code looks tags up by ID, so clusters get -1 and are never matched.
        public static int idOf(AprilTagDetection detection) {
            if (detection instanceof AprilTagSingleDetection) {
                return ((AprilTagSingleDetection) detection).id;
            }
            return -1;
        }

        public static AprilTagDetection getAprilTagByID(int id) {
            for(AprilTagDetection detection: getAprilTagDetections()) {
                if(idOf(detection) == id) {
                    return detection;
                }
            }
            return null;
        }

        public static boolean isValidAprilTag(AprilTagDetection detection) {
            if (detection instanceof AprilTagSingleDetection) {
                return ((AprilTagSingleDetection) detection).metadata != null;
            }
            if (detection instanceof AprilTagClusterDetection) {
                return ((AprilTagClusterDetection) detection).metadata != null;
            }
            return false;
        }

        public static boolean rebaseRelativeToTag(int id, boolean ignoreCorrectionAmount) {
            AprilTagDetection detection = getAprilTagByID(id);
            if(!isValidAprilTag(detection)) { return false; }

            Position position = detection.robotPose.getPosition();
            double rotation = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

            return RobotPose.setPose(position.y, -position.x, -((rotation + 90) % 360), ignoreCorrectionAmount);
        }

        public static List<Integer> getAprilTagIDs() {
            List<Integer> IDs = new ArrayList<>();
            for (AprilTagDetection detection: getAprilTagDetections()) {
                if (idOf(detection) >= 0) IDs.add(idOf(detection));
            }
            return IDs;
        }

        public static double getDistanceFromTag(int id) {
            AprilTagDetection detection = getAprilTagByID(id);
            if(!isValidAprilTag(detection)) { return -1; }

            return detection.ftcPose.range;
        }
        public static double getAngleFromTag(int id) {
            AprilTagDetection detection = getAprilTagByID(id);
            if (!isValidAprilTag(detection)) { return -1; }
            double yawDeg = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
            yawDeg = ((yawDeg * -1) + 90.0);

            return yawDeg;
        }

        public static double getAngleFromTag2(int id) {
            AprilTagDetection detection = getAprilTagByID(id);
            if (!isValidAprilTag(detection)) { return Double.NaN; }

            double yawDeg = Double.NaN;
            if (detection.robotPose != null && detection.robotPose.getOrientation() != null) {
                yawDeg = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
            } else {
                // fallback: no orientation available
                return Double.NaN;
            }

            // original logic: invert and add 90
            double angle = (-yawDeg) + 90.0;

            // normalize to [-180, 180]
            angle = ((angle + 180.0) % 360.0);
            if (angle < 0) angle += 360.0;
            angle = angle - 180.0;

            return angle;
        }
        public static int getClosestAprilTagID() {
            List<AprilTagDetection> dets = getAprilTagDetections();
            double bestRange = Double.POSITIVE_INFINITY;
            int bestId = -1;
            for (AprilTagDetection d : dets) {
                if (d == null || idOf(d) < 0) continue;
                double range = Double.NaN;
                if (d.ftcPose != null && !Double.isNaN(d.ftcPose.range)) {
                    range = d.ftcPose.range;
                } else if (d.robotPose != null && d.robotPose.getPosition() != null) {
                    Position p = d.robotPose.getPosition();
                    range = Math.hypot(p.x, p.y);
                }
                if (Double.isFinite(range) && range < bestRange) {
                    bestRange = range;
                    bestId = idOf(d);
                }
            }
            return bestId;
        }
        // * -1 and then +/- 90
    }
}