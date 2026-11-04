package org.firstinspires.ftc.teamcode.panelsdemo

import com.bylazar.field.PanelsField
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

/**
 * Hardware-free smoke test for the whole Panels + Kotlin setup.
 *
 * It requires NO robot configuration, so you can run it on a bare Control Hub
 * to confirm that:
 *   - Kotlin compiles and the OpMode is registered,
 *   - Panels telemetry reaches the browser,
 *   - the Graph tab picks up numeric values,
 *   - the Field tab receives drawings,
 *   - Configurables can be tuned live (see PanelsDemoConfig).
 *
 * Open http://192.168.43.1:8001 on a laptop joined to the Control Hub's WiFi.
 */
@TeleOp(name = "Panels Kotlin Demo", group = "Panels")
class PanelsKotlinDemo : OpMode() {

    private val panels = PanelsTelemetry.telemetry
    private val panelsField = PanelsField.field
    private val timer = ElapsedTime()

    override fun init() {
        // Choose the coordinate convention drawn on the Field tab.
        panelsField.setOffsets(PanelsField.presets.DEFAULT_FTC)
        panels.debug("Panels Kotlin Demo ready.")
        panels.update(telemetry)
    }

    override fun start() {
        timer.reset()
    }

    override fun loop() {
        val t = timer.seconds()
        val w = 2 * PI * PanelsDemoConfig.waveFrequencyHz
        val a = PanelsDemoConfig.waveAmplitude

        val sinValue = a * sin(w * t)
        val cosValue = a * cos(w * t)

        // The Graph tab plots any telemetry entry of the form <name>:<number>.
        panels.addData("sin", sinValue)
        panels.addData("cos", cosValue)
        panels.addData("elapsed", t)

        // Gamepad values, so you can confirm the Gamepad plugin end to end.
        panels.addData("leftStickY", -gamepad1.left_stick_y.toDouble())
        panels.addData("rightStickX", gamepad1.right_stick_x.toDouble())

        if (PanelsDemoConfig.showDebugLines) {
            panels.debug("Kotlin is alive", "t = %.2f s".format(t))
        }

        // Draw a dot orbiting the middle of the field.
        val r = PanelsDemoConfig.circleRadiusIn
        panelsField.setStyle(PanelsField.RED, PanelsField.BLUE, 1.0)
        panelsField.moveCursor(24.0 * cos(w * t), 24.0 * sin(w * t))
        panelsField.circle(r)
        panelsField.update()

        // Passing the FTC telemetry mirrors the same data to the Driver Hub.
        panels.update(telemetry)
    }
}
