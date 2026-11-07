package org.firstinspires.ftc.teamcode.calibration.runner;

import java.util.LinkedHashMap;
import java.util.Map;

/** A picture the web page draws next to a prompt, for example which wheel should be spinning. */
public final class Display {

    final Map<String, Object> data = new LinkedHashMap<>();

    private Display(String kind) {
        data.put("kind", kind);
    }

    public static final class FourWheelBot {
        public enum Wheel { FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT }

        private FourWheelBot() { }
    }

    /** The robot seen from above with one wheel highlighted. */
    public static Display fourWheelBot(FourWheelBot.Wheel wheel, boolean reversed) {
        Display d = new Display("fourWheelBot");
        d.data.put("wheel", wheel.name());
        d.data.put("reversed", reversed);
        return d;
    }

    /** The robot seen from above with an arrow: "forward", "left", "right", "back", "turn". */
    public static Display arrow(String direction) {
        Display d = new Display("arrow");
        d.data.put("direction", direction);
        return d;
    }
}
