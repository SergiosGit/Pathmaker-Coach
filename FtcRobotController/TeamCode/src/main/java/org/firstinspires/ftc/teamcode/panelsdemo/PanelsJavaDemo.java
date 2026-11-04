package org.firstinspires.ftc.teamcode.panelsdemo;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * The same idea in Java, proving Java and Kotlin sources coexist in TeamCode
 * and that Java can read the Kotlin @Configurable values.
 */
@TeleOp(name = "Panels Java Demo", group = "Panels")
public class PanelsJavaDemo extends OpMode {

    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        panels.debug("Panels Java Demo ready.");
        panels.update(telemetry);
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        double t = timer.seconds();
        double value = PanelsDemoConfig.waveAmplitude
                * Math.sin(2 * Math.PI * PanelsDemoConfig.waveFrequencyHz * t);

        panels.addData("javaWave", value);
        panels.addData("javaElapsed", t);
        panels.update(telemetry);
    }
}
