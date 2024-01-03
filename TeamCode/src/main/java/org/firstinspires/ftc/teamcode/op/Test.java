package org.firstinspires.ftc.teamcode.op;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hw.DriveTrain;


@Autonomous
public class Test extends LinearOpMode {


    DriveTrain driveTrain = new DriveTrain(this);
    double power = 0;
    int endTime = 0;
    int startTime = 0;
    int rotateTime = 4000;

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain.init();
        waitForStart();

        if (opModeIsActive()) {

            startTime = (int) getRuntime();
            telemetry.addData("Time in Seconds: ", startTime);
            telemetry.update();
            power = 0.7;
            driveTrain.setMotorPowers(power, power, power, power); // forward
            sleep(1500);
            driveTrain.setMotorPowers(0,0,0,0);
            sleep(500);
            driveTrain.setMotorPowers(-power, power, -power, power); // strafe left
            sleep(1500);
            driveTrain.setMotorPowers(0,0,0,0);
            sleep(500);

            driveTrain.setMotorPowers(-0.25, -0.25, 0.25, 0.25); // Turn left
            sleep(rotateTime);
            driveTrain.setMotorPowers(0,0,0,0);
            sleep(500);
            driveTrain.setMotorPowers(0.25, 0.25, -0.25, -0.25); //Turn right
            sleep(rotateTime);
            driveTrain.setMotorPowers(0,0,0,0);
            sleep(500);

            driveTrain.setMotorPowers(-power, -power, -power, -power); // backwards
            sleep(1500);
            driveTrain.setMotorPowers(0,0,0,0);
            sleep(500);
            driveTrain.setMotorPowers(power, -power, power, -power); // strafe right
            sleep(1500);
            driveTrain.setMotorPowers(0,0,0,0);
            endTime = (int) getRuntime();
            telemetry.addData("Time in Seconds: ", endTime - startTime);
            telemetry.update();
            sleep(10000);

        }


    }
}
