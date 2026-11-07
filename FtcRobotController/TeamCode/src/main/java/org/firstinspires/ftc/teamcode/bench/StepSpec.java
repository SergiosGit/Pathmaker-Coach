package org.firstinspires.ftc.teamcode.bench;

/**
 * One segment as the PathMaker engines see it: where it starts, where it ends, the heading mode,
 * and whether to stop. Curves are split into several pass-through segments along the curve.
 * Pedro field coordinates, degrees.
 */
public final class StepSpec {
    public final double startX, startY, startH, x, y, h;
    public final Route.Heading heading;
    public final boolean passThrough;
    public final double delayXMs, delayYMs, delayHMs;
    /** Index of the route step this segment belongs to. */
    public final int routeStep;

    public StepSpec(double startX, double startY, double startH, double x, double y, double h,
                    Route.Heading heading, boolean passThrough, double delayXMs, double delayYMs, double delayHMs,
                    int routeStep) {
        this.startX = startX;
        this.startY = startY;
        this.startH = startH;
        this.x = x;
        this.y = y;
        this.h = h;
        this.heading = heading;
        this.passThrough = passThrough;
        this.delayXMs = delayXMs;
        this.delayYMs = delayYMs;
        this.delayHMs = delayHMs;
        this.routeStep = routeStep;
    }
}
