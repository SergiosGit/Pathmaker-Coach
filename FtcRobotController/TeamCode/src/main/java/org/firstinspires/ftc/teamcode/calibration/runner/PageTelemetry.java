package org.firstinspires.ftc.teamcode.calibration.runner;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Live values shown on the web page while a run is active. Numbers are also plotted.
 * Has the telemetry calls Pedro's procedures use.
 */
public final class PageTelemetry {

    private final Map<String, Object> pending = new LinkedHashMap<>();

    public PageTelemetry addData(String caption, Object value) {
        pending.put(caption, value);
        return this;
    }

    public PageTelemetry addLine(String line) {
        pending.put(line, "");
        return this;
    }

    public void update() {
        Map<String, Object> copy = new LinkedHashMap<>(pending);
        pending.clear();
        CalibrationSession.get().live(copy);
    }
}
