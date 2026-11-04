package org.firstinspires.ftc.teamcode.panelsdemo

import com.bylazar.configurables.annotations.Configurable

/**
 * Anything in a @Configurable companion object shows up in the Panels
 * "Configurables" tab and can be edited live from the browser, with no rebuild.
 *
 * @JvmField is required: Panels reads/writes the field directly, so a Kotlin
 * property with getters/setters would not be picked up.
 */
@Configurable
class PanelsDemoConfig {
    companion object {
        @JvmField
        var waveFrequencyHz: Double = 0.5

        @JvmField
        var waveAmplitude: Double = 1.0

        @JvmField
        var circleRadiusIn: Double = 6.0

        @JvmField
        var showDebugLines: Boolean = true
    }
}
