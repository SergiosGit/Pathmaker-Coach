package org.firstinspires.ftc.teamcode.op;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.configuration.RobotConfig;
import org.firstinspires.ftc.teamcode.hw.DriveTrain;
import org.firstinspires.ftc.teamcode.hw.ExtraStuff;
import org.firstinspires.ftc.teamcode.hw.MyIMU;
import org.firstinspires.ftc.teamcode.hw.VisionSystem;
import org.firstinspires.ftc.teamcode.pathmaker.PathDetails;
import org.firstinspires.ftc.teamcode.pathmaker.PathMakerStateMachine;
import org.firstinspires.ftc.teamcode.pathmaker.PathManager;

@Configurable
@Autonomous (name="Close Red Auto")
public class AutoOp_RedClose extends LinearOpMode {

    public static double thisForwardPower = 0;
    public static double thisStrafePower = 0;
    public static double thisTurnPower = 0.2;
    public static double thisHeadingDrive = 0;
    public static int thisPathNumber = 1;
    public static int runTest_ms = 100;

    private int cycles = 0;
    private final int TEST_CYCLES = 0;


    DriveTrain driveTrain = new DriveTrain(this);
    public DcMotorEx intakeMotor;
    public DcMotorEx outtakeMotor;
    public Servo intakeServo;
    ExtraStuff extraStuff = new ExtraStuff(intakeMotor, outtakeMotor, intakeServo);
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        RobotConfig.initializeRobotType(hardwareMap.appContext, true);

        if(RobotConfig.Camera.useCamera()) {
            if(RobotConfig.Camera.multipleCameras()) {
                VisionSystem.createVisionSystem(hardwareMap.get(WebcamName.class, RobotConfig.Camera.getWebcamOneName()),
                        hardwareMap.get(WebcamName.class, RobotConfig.Camera.getWebcamTwoName()));
            } else {
                VisionSystem.createVisionSystem(hardwareMap.get(WebcamName.class, RobotConfig.Camera.getWebcamOneName()));
            }
        }

        PathDetails.thisAutoStrategy  = PathDetails.AutoType.RedClose;

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtake_motor");
        intakeServo = hardwareMap.get(Servo.class, "intake_servo");
        ExtraStuff extraStuff = new ExtraStuff(intakeMotor, outtakeMotor, intakeServo);
        ExtraStuff.initShooter(hardwareMap);

        RobotPose.initializePose(this, driveTrain, telemetry);
        RobotPose.setPose(0, 0, 0, true);

        MyIMU.init(this);
        MyIMU.updateTelemetry(telemetry);

        PathMakerStateMachine.setAutonomous();
        PathDetails.initAutoPathList();

        telemetry.addData("this path number", thisPathNumber);
        telemetry.update();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        waitForStart();

        if (RobotConfig.Camera.useCamera() && RobotConfig.Camera.enableLivePreview()) {
            PanelsCameraStream.INSTANCE.startStream(VisionSystem.getVisionPortal(), 30);
        }
        while(opModeIsActive()) {

            if(isStopRequested()) { return; }

            // thisPathNumber: -1 and 0 used to test, 1 is normal
            if (thisPathNumber == -1) {
                WheelPowerManager.setDrivePower(driveTrain, thisForwardPower, thisStrafePower, thisTurnPower, thisHeadingDrive);
                sleep(runTest_ms);
                WheelPowerManager.setDrivePower(driveTrain, 0, 0, 0, 0);
                RobotPose.readPose();
            } else if (thisPathNumber == 0) {
                // use dashboard parameters
                //PathManager.moveRobot();
                RobotPose.setPose(0, 0, 0, true);
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_SET_PATH;
                PathMakerStateMachine.currentPath = PathMakerStateMachine.nextPath = 0;
                sleep(runTest_ms);
            } else if(thisPathNumber == 1) {

                updateTelemetry(timer);
                PathMakerStateMachine.updateAuto(telemetry);

            }
        }
    }

    private void updateTelemetry(ElapsedTime timer) {

        cycles++;
        if (cycles > TEST_CYCLES) {

            double cycleTime = timer.milliseconds() / cycles;
            timer.reset();
            telemetry.addData("LeftEncoder: ", DriveTrain.frontLeft.getCurrentPosition());
            telemetry.addData("MiddleEncoder: ", DriveTrain.backRight.getCurrentPosition());
            telemetry.addData("RightEncoder: ", DriveTrain.frontRight.getCurrentPosition());

            telemetry.addData("State", PathMakerStateMachine.pm_state);
            telemetry.addData("PathDetails.currentPath", PathMakerStateMachine.currentPath < 0 ? -1 : PathDetails.autoPathList.get(PathMakerStateMachine.currentPath));
            telemetry.addLine(String.format("PathDetails.elapsedTime_ms %.1f", PathDetails.elapsedTime_ms.milliseconds()));
            telemetry.addLine(String.format("ave/PM/DT cycle %d /  %d (ms) / %.3f (s)", (int) cycleTime, PathManager.PMcycleTime_ms, RobotPose.DT_seconds));

            // position relative to path goals
            telemetry.addLine(String.format("delta-is-should f/s/a %.1f / %.1f / %.1f",
                    PathManager.deltaIsShouldY,
                    PathManager.deltaIsShouldX,
                    PathManager.deltaIsShouldAngle));

            // direction of travel required to reach path goal
            telemetry.addLine(String.format("delta is-should signum f/s/a %d / %d / %d",
                    (int) Math.signum(PathManager.deltaIsShouldY),
                    (int) Math.signum(PathManager.deltaIsShouldX),
                    (int) Math.signum(PathManager.deltaIsShouldAngle)));

            // initial direction of travel
            telemetry.addLine(String.format("initial power signum f/s/a %d / %d / %d",
                    (int) Math.signum(PathDetails.yInitialPowerSignum),
                    (int) Math.signum(PathDetails.xInitialPowerSignum),
                    (int) Math.signum(PathDetails.aInitialPowerSignum)));

            // powers in each DOF
            telemetry.addLine(String.format("power x,y,a %.2f / %.2f / %.2f",
                    PathManager.xPower,
                    PathManager.yPower,
                    PathManager.turnPower));

            // robot pose
            telemetry.addLine(String.format("RoboPose (y,x,a) f/s/t %.1f / %.1f / %.1f (in/deg)",
                    RobotPose.getFieldY_in(),
                    RobotPose.getFieldX_in(),
                    RobotPose.getFieldAngle_deg()));

            // robot velocities
            telemetry.addLine(String.format("x/y/a velocity %.1f / %.1f / %.1f (v_r %.3f)",
                    RobotPose.getXVelocity_inPerSec(),
                    RobotPose.getYVelocity_inPerSec(),
                    RobotPose.getHeadingVelocity_degPerSec(),
                    PathManager.v_ramp));

            // shooter
            ExtraStuff.shooter.addTelemetry(telemetry);

            telemetry.update();
            cycles = 0;
        }
    }
}
