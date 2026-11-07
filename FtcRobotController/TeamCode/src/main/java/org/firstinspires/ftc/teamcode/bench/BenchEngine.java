package org.firstinspires.ftc.teamcode.bench;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.api.Paths;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathTracker;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Drives a {@link Route} with one of three engines, all through Pedro's Follower so they share
 * the localizer, the motor output and the loop: only the control law differs.
 *
 * Non-blocking: call {@link #update(double)} once per OpMode loop.
 */
public class BenchEngine {

    public enum Kind { PEDRO, PATHMAKER1, PATHMAKER2 }

    /** Curves are split into this many segments for the PathMaker engines. */
    static final int CURVE_SEGMENTS = 4;

    public final Kind kind;
    private final Follower follower;
    private final RecordingDrivetrain drive;
    private final RecordingAlgorithm algorithm;
    private final PathMakerAlgorithm pathMaker; // null for Pedro
    private final List<Path> groups = new ArrayList<>();
    private final List<List<StepSpec>> groupSpecs = new ArrayList<>();
    /** Pedro turn-in-place targets, by group (null for a normal path). */
    private final List<Pose> turnTargets = new ArrayList<>();
    private int group = -1;
    private boolean done = true;
    private double settledFor = 0;

    /**
     * Pedro has no turn-in-place path, so it holds the new pose. The turn counts as done once the
     * robot has stopped turning near the heading (within 5°) for a moment; the log keeps the error.
     */
    static final double TURN_NEAR_DEG = 5, TURN_STILL_DEG_S = 3, TURN_SETTLE_S = 0.25;

    /**
     * @param speedScale 1 = full speed; 0.6 = the benchmark's "matched" setting
     * @param foresight  Pedro's configuration (only used by PEDRO)
     * @param volts      battery voltage (used by PATHMAKER2)
     */
    public BenchEngine(Kind kind, Localizer localizer, Drivetrain drivetrain, double speedScale,
                       Supplier<ForesightConfig> foresight, DoubleSupplier volts) {
        this.kind = kind;
        this.drive = new RecordingDrivetrain(drivetrain);
        Algorithm inner;
        switch (kind) {
            case PEDRO: {
                ForesightConfig cfg = foresight.get();
                if (speedScale < 1) cfg.maxPathSpeed.set(speedScale);
                inner = new Foresight(cfg);
                pathMaker = null;
                break;
            }
            case PATHMAKER1: {
                PathMaker1Algorithm a = new PathMaker1Algorithm();
                a.powerScaling = speedScale;
                inner = pathMaker = a;
                break;
            }
            default:
                inner = pathMaker = new PathMaker2Algorithm(speedScale, volts);
        }
        this.algorithm = new RecordingAlgorithm(inner);
        this.follower = new Follower(localizer, drive, algorithm);
    }

    public Follower follower() { return follower; }

    public void setPose(double x, double y, double headingDeg) {
        follower.setPose(new Pose(x, y, Math.toRadians(headingDeg)));
    }

    /** Starts the route from the beginning. */
    public void start(Route route) {
        groups.clear();
        groupSpecs.clear();
        turnTargets.clear();
        build(route);
        group = -1;
        done = false;
        next();
    }

    /** One loop. Moves on to the next part of the route when a part stops. */
    public void update(double dt) {
        follower.update(dt);
        if (done) return;
        boolean partDone;
        Pose turn = turnTargets.get(group);
        if (turn != null) {
            double err = Route.wrapDeg(Math.toDegrees(turn.heading() - follower.pose().heading()));
            boolean still = Math.abs(Math.toDegrees(follower.twist().omega)) < TURN_STILL_DEG_S;
            settledFor = Math.abs(err) < TURN_NEAR_DEG && still ? settledFor + dt : 0;
            partDone = settledFor >= TURN_SETTLE_S;
        } else {
            partDone = !follower.following();
        }
        if (partDone) {
            if (group + 1 < groups.size()) next();
            else done = true;
        }
    }

    /** True once the last step has ended. The follower keeps holding the end pose (except PathMaker 1). */
    public boolean done() { return done; }

    public Pose pose() { return follower.pose(); }

    public MotionState state() { return MotionState.ofTwist(follower.pose(), follower.twist()); }

    public DrivePowers lastPowers() { return drive.last; }

    /** Route step being driven, or -1 before the start, or the number of steps when done. */
    public int routeStep() {
        if (group < 0) return -1;
        if (done) return countSteps();
        List<StepSpec> specs = groupSpecs.get(group);
        int i = Math.min(algorithm.segment, specs.size() - 1);
        return specs.get(Math.max(i, 0)).routeStep;
    }

    public boolean atParametricEnd() { return algorithm.atParametricEnd(); }

    public Map<String, Object> debug() { return algorithm.debug(); }

    // ---- building Pedro paths from the route

    private void build(Route r) {
        List<Path> segs = new ArrayList<>();
        List<StepSpec> specs = new ArrayList<>();
        for (int i = 0; i < r.steps.size(); i++) {
            Route.Step s = r.steps.get(i);
            double[] a = r.startOf(i);
            if (s.isCurve() && kind != Kind.PEDRO) {
                // PathMaker drives a curve as pass-through points along it, facing along the curve
                double[] prev = a;
                for (int k = 1; k <= CURVE_SEGMENTS; k++) {
                    double t = (double) k / CURVE_SEGMENTS;
                    double[] pt = Route.bezier(new double[]{a[0], a[1]}, s.control, new double[]{s.x, s.y}, t);
                    boolean last = k == CURVE_SEGMENTS;
                    double h = last ? s.headingDeg : Math.toDegrees(Math.atan2(pt[1] - prev[1], pt[0] - prev[0]));
                    StepSpec sp = new StepSpec(prev[0], prev[1], prev[2], pt[0], pt[1], h,
                            last ? Route.Heading.LINEAR : Route.Heading.TANGENT, !last || s.passThrough, 0, 0, 0, i);
                    specs.add(sp);
                    segs.add(line(sp));
                    prev = new double[]{pt[0], pt[1], h};
                }
            } else {
                StepSpec sp = new StepSpec(a[0], a[1], a[2], s.x, s.y, s.headingDeg, s.heading, s.passThrough,
                        s.delayXMs, s.delayYMs, s.delayHMs, i);
                specs.add(sp);
                boolean zero = Math.hypot(s.x - a[0], s.y - a[1]) < 1e-6;
                // PathMaker only uses the end point; Pedro refuses a zero-length line, so give it a tiny one
                segs.add(zero && kind == Kind.PEDRO ? null : s.isCurve() ? curve(a, s)
                        : zero ? line(new StepSpec(a[0] - 0.01, a[1], a[2], s.x, s.y, s.headingDeg, s.heading,
                                false, 0, 0, 0, i)) : pedroLine(a, s));
            }
            if (!s.passThrough) { // a stop ends a part of the route
                boolean turnInPlace = kind == Kind.PEDRO && segs.size() == 1
                        && Math.hypot(s.x - a[0], s.y - a[1]) < 1e-6;
                turnTargets.add(turnInPlace ? new Pose(s.x, s.y, Math.toRadians(s.headingDeg)) : null);
                groups.add(turnInPlace ? null : segs.size() == 1 ? segs.get(0) : Paths.path(segs.toArray(new Path[0])));
                groupSpecs.add(new ArrayList<>(specs));
                segs.clear();
                specs.clear();
            }
        }
    }

    private static Path pedroLine(double[] a, Route.Step s) {
        Path p = Paths.line(new Pose(a[0], a[1], Math.toRadians(a[2])), new Pose(s.x, s.y, Math.toRadians(s.headingDeg)));
        return s.heading == Route.Heading.LINEAR
                ? p.linear(Math.toRadians(a[2]), Math.toRadians(s.headingDeg))
                : p.constant(Math.toRadians(s.headingDeg));
    }

    private static Path curve(double[] a, Route.Step s) {
        return Paths.curve(Vector2D.cartesian(a[0], a[1]), Vector2D.cartesian(s.control[0], s.control[1]),
                Vector2D.cartesian(s.x, s.y)).tangent();
    }

    private static Path line(StepSpec sp) {
        return Paths.line(new Pose(sp.startX, sp.startY, Math.toRadians(sp.startH)),
                new Pose(sp.x, sp.y, Math.toRadians(sp.h))).constant(Math.toRadians(sp.h));
    }

    private void next() {
        group++;
        settledFor = 0;
        if (pathMaker != null) pathMaker.setSpecs(groupSpecs.get(group));
        if (turnTargets.get(group) != null) follower.hold(turnTargets.get(group));
        else follower.follow(groups.get(group));
    }

    private int countSteps() {
        int n = 0;
        for (List<StepSpec> g : groupSpecs) for (StepSpec s : g) n = Math.max(n, s.routeStep + 1);
        return n;
    }

    // ---- recording wrappers, for the log

    /** Remembers the last powers sent to the motors. */
    static final class RecordingDrivetrain implements Drivetrain {
        final Drivetrain inner;
        DrivePowers last = DrivePowers.zero();

        RecordingDrivetrain(Drivetrain inner) { this.inner = inner; }

        @Override public void drive(DrivePowers p, boolean b) { last = p; inner.drive(p, b); }
        @Override public double maxScaling(DrivePowers a, DrivePowers b) { return inner.maxScaling(a, b); }
        @Override public void stop() { last = DrivePowers.zero(); inner.stop(); }
        @Override public void stop(boolean b) { last = DrivePowers.zero(); inner.stop(b); }
        @Override public Map<String, Object> debug() { return inner.debug(); }
        @Override public double interpolateVelocity(double a, double b, double c) { return inner.interpolateVelocity(a, b, c); }
        @Override public double interpolateAcceleration(double a, double b, double c) { return inner.interpolateAcceleration(a, b, c); }
    }

    /** Remembers which segment of the path the engine is on. */
    static final class RecordingAlgorithm implements Algorithm {
        final Algorithm inner;
        int segment = 0;

        RecordingAlgorithm(Algorithm inner) { this.inner = inner; }

        @Override
        public DrivePowers calculatePath(Drivetrain d, PathTracker t, MotionState s, double dt) {
            segment = t.currentIndex();
            return inner.calculatePath(d, t, s, dt);
        }

        @Override
        public DrivePowers calculateHold(Drivetrain d, Pose p, MotionState s, boolean scale, double dt) {
            return inner.calculateHold(d, p, s, scale, dt);
        }

        @Override public double completion() { return inner.completion(); }
        @Override public Pose closestPose() { return inner.closestPose(); }
        @Override public Vector2D closestTangent() { return inner.closestTangent(); }
        @Override public Vector2D closestNormal() { return inner.closestNormal(); }
        @Override public double curvature() { return inner.curvature(); }
        @Override public double remainingDistance() { return inner.remainingDistance(); }
        @Override public double parametricCompletion() { return inner.parametricCompletion(); }
        @Override public boolean atParametricEnd() { return inner.atParametricEnd(); }
        @Override public void reset() { segment = 0; inner.reset(); }
        @Override public boolean isBusy() { return inner.isBusy(); }
        @Override public Map<String, Object> debug() { return inner.debug(); }
    }
}
