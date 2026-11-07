package org.firstinspires.ftc.teamcode.bench;

/**
 * The benchmark routes from docs/PATHING-BENCHMARK-PLAN.md, section 5.2. All start at
 * (72, 24) facing up the field (90°): mid-width, one tile from the near wall.
 */
public final class BenchRoutes {

    public enum Test { A1, A2, A3, A4, A5, A6, HOLD }

    public static final double START_X = 72, START_Y = 24, START_H = 90;

    private BenchRoutes() { }

    public static Route get(Test t) {
        Route.Builder b = Route.from(t.name(), START_X, START_Y, START_H);
        switch (t) {
            case A1: // straight forward 48 in
                return b.toConstant(72, 72, 90).build();
            case A2: // strafe right 48 in
                return b.toConstant(120, 24, 90).build();
            case A3: // diagonal while turning to 0°
                return b.to(108, 60, 0).build();
            case A4: // turn in place 180°
                return b.to(72, 24, 270).build();
            case A5: // pass, pass, stop
                return b.through(72, 72, 90).through(108, 96, 90).toConstant(72, 96, 90).build();
            case A6: // quarter circle of radius 36 around (108, 24), facing along the curve
                return b.curve(72, 60, 108, 60, 0).build();
            case HOLD: // stay at the start (loop time and push tests)
            default:
                return b.toConstant(START_X, START_Y, START_H).build();
        }
    }
}
