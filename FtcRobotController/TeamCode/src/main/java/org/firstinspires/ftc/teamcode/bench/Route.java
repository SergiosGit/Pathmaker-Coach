package org.firstinspires.ftc.teamcode.bench;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * One route, written once and driven by any engine (Pedro, PathMaker 1, PathMaker 2).
 *
 * Pedro field coordinates: inches from the field corner, +x right, +y up the field, heading in
 * degrees counterclockwise with 0 facing +x. Plain Java, so routes can be checked on a laptop.
 */
public final class Route {

    public enum Heading { CONSTANT, LINEAR, TANGENT }

    /** One move. A route stops at the end of a step unless the step passes through. */
    public static final class Step {
        public final double x, y, headingDeg;
        public final Heading heading;
        public final boolean passThrough;
        /** Bézier control point for a curve, or null for a straight line. */
        public final double[] control;
        /** PathMaker 1 only: per-axis start delays after the step begins. Pedro ignores them. */
        public final double delayXMs, delayYMs, delayHMs;

        Step(double x, double y, double headingDeg, Heading heading, boolean passThrough, double[] control,
             double delayXMs, double delayYMs, double delayHMs) {
            this.x = x;
            this.y = y;
            this.headingDeg = headingDeg;
            this.heading = heading;
            this.passThrough = passThrough;
            this.control = control;
            this.delayXMs = delayXMs;
            this.delayYMs = delayYMs;
            this.delayHMs = delayHMs;
        }

        public boolean isCurve() { return control != null; }
    }

    public final String name;
    public final double startX, startY, startHeadingDeg;
    public final List<Step> steps;

    private Route(String name, double x, double y, double h, List<Step> steps) {
        this.name = name;
        this.startX = x;
        this.startY = y;
        this.startHeadingDeg = h;
        this.steps = Collections.unmodifiableList(new ArrayList<>(steps));
    }

    public static Builder from(String name, double x, double y, double headingDeg) {
        return new Builder(name, x, y, headingDeg);
    }

    /** The pose the robot should end at. */
    public double[] endPose() {
        if (steps.isEmpty()) return new double[]{startX, startY, startHeadingDeg};
        Step s = steps.get(steps.size() - 1);
        return new double[]{s.x, s.y, s.headingDeg};
    }

    /** Start pose of step i: the start of the route or the end of the step before. */
    public double[] startOf(int i) {
        if (i == 0) return new double[]{startX, startY, startHeadingDeg};
        Step p = steps.get(i - 1);
        return new double[]{p.x, p.y, p.headingDeg};
    }

    /** The route mirrored to the other alliance: x → 144 − x, heading → 180° − heading. */
    public Route mirrored(String newName) {
        Builder b = from(newName, 144 - startX, startY, 180 - startHeadingDeg);
        for (Step s : steps) {
            double[] c = s.control == null ? null : new double[]{144 - s.control[0], s.control[1]};
            b.steps.add(new Step(144 - s.x, s.y, 180 - s.headingDeg, s.heading, s.passThrough, c,
                    s.delayXMs, s.delayYMs, s.delayHMs));
        }
        return b.build();
    }

    public static final class Builder {
        private final String name;
        private final double x, y, h;
        private final List<Step> steps = new ArrayList<>();

        Builder(String name, double x, double y, double h) {
            this.name = name;
            this.x = x;
            this.y = y;
            this.h = h;
        }

        /** Straight line, heading turns linearly to the new heading, then stop. */
        public Builder to(double x, double y, double headingDeg) {
            return add(x, y, headingDeg, Heading.LINEAR, false, null);
        }

        /** Straight line keeping the given heading, then stop. */
        public Builder toConstant(double x, double y, double headingDeg) {
            return add(x, y, headingDeg, Heading.CONSTANT, false, null);
        }

        /** Straight line, heading constant, without stopping at the end. */
        public Builder through(double x, double y, double headingDeg) {
            return add(x, y, headingDeg, Heading.CONSTANT, true, null);
        }

        /** Quadratic Bézier curve through a control point, heading along the curve, then stop. */
        public Builder curve(double cx, double cy, double x, double y, double endHeadingDeg) {
            return add(x, y, endHeadingDeg, Heading.TANGENT, false, new double[]{cx, cy});
        }

        /** PathMaker 1 per-axis delays for the last step added. */
        public Builder delays(double xMs, double yMs, double hMs) {
            Step s = steps.remove(steps.size() - 1);
            steps.add(new Step(s.x, s.y, s.headingDeg, s.heading, s.passThrough, s.control, xMs, yMs, hMs));
            return this;
        }

        private Builder add(double x, double y, double h, Heading mode, boolean pass, double[] control) {
            steps.add(new Step(x, y, h, mode, pass, control, 0, 0, 0));
            return this;
        }

        public Route build() {
            if (!steps.isEmpty() && steps.get(steps.size() - 1).passThrough) {
                throw new IllegalStateException("Route " + name + " must end with a step that stops");
            }
            return new Route(name, x, y, h, steps);
        }
    }

    // ---- geometry shared by the engines ----

    /** Point on the quadratic Bézier from a through control c to b, at t in [0, 1]. */
    public static double[] bezier(double[] a, double[] c, double[] b, double t) {
        double u = 1 - t;
        return new double[]{u * u * a[0] + 2 * u * t * c[0] + t * t * b[0],
                u * u * a[1] + 2 * u * t * c[1] + t * t * b[1]};
    }

    /** Angle difference b − a in degrees, wrapped to (−180, 180]. */
    public static double wrapDeg(double d) {
        d = ((d + 180) % 360 + 360) % 360 - 180;
        return d == -180 ? 180 : d;
    }
}
