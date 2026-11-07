package org.firstinspires.ftc.teamcode.bench;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.PathTracker;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Lets a PathMaker control law drive inside Pedro's Follower, so it uses exactly the same
 * localizer, motor output and loop as Pedro. Pedro gives it the path segment; PathMaker only
 * uses the segment's end point (and the StepSpec with stop/pass and delays), not its shape.
 */
public abstract class PathMakerAlgorithm implements Algorithm {

    private List<StepSpec> specs;
    private int lastIndex = -1;
    private long stepStartNanos;
    protected double remaining = 0;
    protected boolean atEnd = false;
    protected final Map<String, Object> debug = new LinkedHashMap<>();

    /** Segments of the path that is about to be followed, in order. */
    public void setSpecs(List<StepSpec> specs) {
        this.specs = specs;
        lastIndex = -1;
    }

    /** Called once when a segment starts. */
    protected abstract void startStep(StepSpec s, MotionState state);

    /** Robot-relative powers for this loop; sets {@link #atEnd} when the segment is complete. */
    protected abstract DrivePowers step(StepSpec s, MotionState state, double elapsedMs, double dt);

    protected abstract DrivePowers hold(Pose target, MotionState state, double dt);

    @Override
    public DrivePowers calculatePath(Drivetrain drivetrain, PathTracker tracker, MotionState state, double dt) {
        int i = tracker.currentIndex();
        StepSpec s = specs.get(Math.min(i, specs.size() - 1));
        if (i != lastIndex) {
            lastIndex = i;
            stepStartNanos = System.nanoTime();
            atEnd = false;
            startStep(s, state);
        }
        DrivePowers p = step(s, state, (System.nanoTime() - stepStartNanos) / 1e6, dt);
        if (atEnd) tracker.advance();
        return p;
    }

    @Override
    public DrivePowers calculateHold(Drivetrain drivetrain, Pose pose, MotionState state, boolean useHoldScaling, double dt) {
        return hold(pose, state, dt);
    }

    @Override public double completion() { return atEnd ? 1 : 0; }
    @Override public Pose closestPose() { return Pose.zero(); }
    @Override public Vector2D closestTangent() { return Vector2D.zero(); }
    @Override public Vector2D closestNormal() { return Vector2D.zero(); }
    @Override public double curvature() { return 0; }
    @Override public double remainingDistance() { return remaining; }
    @Override public boolean atParametricEnd() { return atEnd; }
    @Override public void reset() { lastIndex = -1; atEnd = false; }
    @Override public boolean isBusy() { return !atEnd; }
    @Override public Map<String, Object> debug() { return debug; }

    // ---- helpers

    /** Field-relative powers (x right, y up the field, counterclockwise) to robot-relative ones. */
    static DrivePowers toRobot(double px, double py, double turn, double headingRad) {
        double c = Math.cos(headingRad), s = Math.sin(headingRad);
        return new DrivePowers(px * c + py * s, -px * s + py * c, turn);
    }

    static double deg(double rad) { return Math.toDegrees(rad); }
}
