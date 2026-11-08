package org.firstinspires.ftc.teamcode.calibration.runner;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Buttons and editable values the web page shows while a run is active, for runs the person
 * steers from the page (the shooter tuning step). A button press reaches the run through
 * {@link TuningOpMode#nextControl()}; an edited value is saved like any value typed on the page,
 * so a run that reads it (through its field, e.g. ShooterConfig.kP) uses it in the next loop.
 */
public final class Controls {

    final Map<String, Object> data = new LinkedHashMap<>();
    private final List<Object> buttons = new ArrayList<>();

    public Controls() {
        data.put("buttons", buttons);
        data.put("keys", new ArrayList<>());
        data.put("status", "");
    }

    public Controls button(String id, String label, boolean enabled, boolean primary) {
        Map<String, Object> b = new LinkedHashMap<>();
        b.put("id", id);
        b.put("label", label);
        b.put("enabled", enabled);
        b.put("primary", primary);
        buttons.add(b);
        return this;
    }

    public Controls button(String id, String label) { return button(id, label, true, false); }

    /** Calibration keys shown as editable boxes next to the buttons. */
    public Controls keys(String... ids) {
        data.put("keys", new ArrayList<Object>(Arrays.asList((Object[]) ids)));
        return this;
    }

    /** A line of text above the buttons, e.g. what the run is doing now. */
    public Controls status(String text) {
        data.put("status", text);
        return this;
    }
}
