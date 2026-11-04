package org.firstinspires.ftc.teamcode.op;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.camerastream.PanelsCameraStream;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

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

@TeleOp (name="Field Centric Driving")
public class TeleOpMode extends LinearOpMode {

    private DriveTrain driveTrain = new DriveTrain(this);

    private int cycles = 0;
    private final int TEST_CYCLES = 10;
    public DcMotorEx intakeMotor;
    public DcMotorEx outtakeMotor;
    public Servo intakeServo;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        RobotConfig.initializeRobotType(hardwareMap.appContext, false);
        if(RobotConfig.Camera.useCamera()) {
            if(RobotConfig.Camera.multipleCameras()) {
                VisionSystem.createVisionSystem(hardwareMap.get(WebcamName.class, RobotConfig.Camera.getWebcamOneName()),
                        hardwareMap.get(WebcamName.class, RobotConfig.Camera.getWebcamTwoName()));
            } else {
                VisionSystem.createVisionSystem(hardwareMap.get(WebcamName.class, RobotConfig.Camera.getWebcamOneName()));
            }
        }

        RobotPose.initializePose(this, driveTrain, telemetry);
        RobotPose.setPose(0, 0, 0, true);
        MyIMU.init(this);
        PathMakerStateMachine.setDriverControlled();
        PathDetails.initializePath();
        MyIMU.updateTelemetry(telemetry);
        telemetry.update();

        //Extra Stuff for later
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtake_motor");
        intakeServo = hardwareMap.get(Servo.class, "intake_servo");
        ExtraStuff extraStuff = new ExtraStuff(intakeMotor, outtakeMotor, intakeServo);
        ExtraStuff.initShooter(hardwareMap);

        waitForStart();

        if (RobotConfig.Camera.useCamera() && RobotConfig.Camera.enableLivePreview()) {
            PanelsCameraStream.INSTANCE.startStream(VisionSystem.getVisionPortal(), 30);
        }

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while(opModeIsActive()) {

            if(isStopRequested()) { return; }

            if (gamepad1.a) {// reset robot orientation
                RobotPose.initializePose(this, driveTrain, telemetry);
            }

            PathMakerStateMachine.updateTele(gamepad1, telemetry);
            extraStuff.getGamepadInput(gamepad1, gamepad2);


            updateTelemetry(timer);
        }
    }

    private void updateTelemetry(ElapsedTime timer) throws InterruptedException {
        cycles++;

        //if(cycles > TEST_CYCLES) {

        double t1 = timer.milliseconds() / cycles;
        timer.reset();

        telemetry.addData("State", PathMakerStateMachine.pm_state);
        telemetry.addData("PathDetails.currentPath", PathMakerStateMachine.currentPath < 0 ? -1 : PathDetails.autoPathList.get(PathMakerStateMachine.currentPath));
        telemetry.addLine(String.format("PathDetails.elapsedTime_ms %.1f", PathDetails.elapsedTime_ms.milliseconds()));
        telemetry.addLine(String.format("ave tele/PM cycle %d /  %d (ms)", (int) t1, PathManager.PMcycleTime_ms));
        //
        // robot pose
        //
        telemetry.addLine(String.format("RoboPose x/y/a %.1f / %.1f / %.1f (in/deg)",
                RobotPose.getFieldX_in(),
                RobotPose.getFieldY_in(),
                RobotPose.getFieldAngle_deg()));
        //
        // odometry stuff
        //
        telemetry.addData("LeftEncoder: ", DriveTrain.frontLeft.getCurrentPosition());
        telemetry.addData("MiddleEncoder: ", DriveTrain.backRight.getCurrentPosition());
        telemetry.addData("RightEncoder: ", DriveTrain.frontRight.getCurrentPosition());
        //
        // power x/y/a
        //
        telemetry.addLine(String.format("power Y/X/A %.2f / %.2f / %.2f",
                PathMakerStateMachine.yPower,
                PathMakerStateMachine.xPower,
                PathMakerStateMachine.turnPower));
        //
        // outake Stuff
        //
        telemetry.addData("intakeMotor Power: ", intakeMotor.getPower());
        ExtraStuff.shooter.addTelemetry(telemetry);
        //
        //
        //
            /* gamepad x/y/a input
            telemetry.addLine(String.format("gamepad Y/X/A %.1f / %.1f / %.1f",
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x));
            */
            /* robot velocity
            telemetry.addLine(String.format("x/y/a velocity %.1f / %.1f / %.1f ",
                    RobotPose.getForwardVelocity_inPerSec(),
                    RobotPose.getStrafeVelocity_inPerSec(),
                    RobotPose.getHeadingVelocity_degPerSec()));
            telemetry.addLine(String.format("actual power Y/X/A %.1f / %.1f / %.1f",
                    PathManager.yPower,
                    PathManager.xPower,
                    PathManager.turnPower));
            telemetry.addLine(String.format("last power Y/X/A %.1f / %.1f / %.1f",
                    PathManager.yPowerLast,
                    PathManager.xPowerLast,
                    PathManager.turnPowerLast));
            telemetry.addLine(String.format("delta is should A %.1f",
                    PathManager.deltaIsShouldAngle));
                    */
            /* Robot Type Debug Vars
            String androidId = Settings.Secure.getString(hardwareMap.appContext.getContentResolver(), Settings.Secure.ANDROID_ID);

            telemetry.addData("Android ID: ", androidId);
            telemetry.addData("robotType: ", RobotConfig.Odometry.robotType);
            telemetry.addData("lateralDistance: ", RobotConfig.Odometry.lateralDistance);
            telemetry.addData("forwardOffset: ", RobotConfig.Odometry.forwardOffset);
*/
        //    MyIMU.updateTelemetry(telemetry);
        telemetry.update();
        cycles = 0;
    }
}
