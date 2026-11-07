package org.firstinspires.ftc.teamcode.calibration.runner;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.calibration.Calibration;

import java.text.SimpleDateFormat;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Date;
import java.util.Deque;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.function.Supplier;

/**
 * Connects the web page (server threads) with the Calibrate OpMode (its own thread).
 *
 * The page asks for a step to run; the OpMode thread runs it. Whenever the step needs the person
 * (a form, a confirmation, Start for a run), the OpMode thread waits here until the page answers.
 * Every answer carries the prompt's id, so a stale click from an old page can't answer a new prompt.
 * All state lives here, on the robot, so the page can be closed and reopened at any time.
 */
public final class CalibrationSession {

    /** The Calibrate OpMode, as seen by the session. */
    public interface Host {
        boolean stopRequested();
        HardwareMap hardwareMap();
        Gamepad gamepad1();
        Gamepad gamepad2();
    }

    /** A step listed on the page. */
    public static final class Step {
        public final String id, group, title, summary;
        public final String[] keys;
        final Supplier<Procedure> factory;

        public Step(String id, String group, String title, String summary, String[] keys, Supplier<Procedure> factory) {
            this.id = id;
            this.group = group;
            this.title = title;
            this.summary = summary;
            this.keys = keys;
            this.factory = factory;
        }
    }

    public enum Phase { IDLE, STARTING, INPUTS, CONFIRM, READY, RUNNING, DONE, FAILED }

    private static final CalibrationSession INSTANCE = new CalibrationSession();
    private static final int LOG_LINES = 80, SAMPLES = 600;

    public static CalibrationSession get() { return INSTANCE; }

    private final List<Step> steps = new ArrayList<>();
    private Host host;
    private Step pending, current;
    private Phase phase = Phase.IDLE;
    private long promptId = 0;
    private String title = "", message = "";
    private Inputs inputs;
    private Map<String, Object> answers;
    private boolean confirmed, startPressed, finishPressed, abortRequested;
    private TuningOpMode<?> run;
    private Map<String, Object> display;
    private Map<String, Object> live = new LinkedHashMap<>();
    private final List<double[]> samples = new ArrayList<>();
    private final List<String> sampleNames = new ArrayList<>();
    private long runStartNanos;
    private final List<Object> results = new ArrayList<>();
    private String outcome = "";
    private final Deque<String> log = new ArrayDeque<>();

    private CalibrationSession() { }

    // ------------------------------------------------------------------ setup

    public synchronized void setSteps(List<Step> list) {
        steps.clear();
        steps.addAll(list);
    }

    public synchronized void attach(Host h) {
        host = h;
        log("Calibrate OpMode started");
        notifyAll();
    }

    public synchronized void detach() {
        host = null;
        pending = null;
        if (current != null) log("Calibrate OpMode stopped");
        notifyAll();
    }

    // ------------------------------------------------------------------ page actions

    /** Asks the OpMode to run a step. Returns an error message, or null. */
    public synchronized String requestRun(String stepId) {
        if (host == null) return "Start the Calibrate OpMode on the Driver Hub first (INIT, then ▶).";
        if (current != null) return "Another step is running.";
        Step s = find(stepId);
        if (s == null) return "Unknown step " + stepId;
        pending = s;
        phase = Phase.STARTING;
        promptId++;
        title = s.title;
        message = "Starting…";
        results.clear();
        outcome = "";
        notifyAll();
        return null;
    }

    public synchronized String answer(long id, String action, Map<String, Object> values) {
        if (id != promptId) return "This prompt was already answered, perhaps in another browser tab. The page now shows the current one.";
        switch (action) {
            case "inputs":
                if (phase != Phase.INPUTS) return "Not waiting for inputs.";
                answers = values;
                break;
            case "confirm":
                if (phase != Phase.CONFIRM) return "Not waiting for a confirmation.";
                confirmed = true;
                break;
            case "start":
                if (phase != Phase.READY) return "Not waiting for Start.";
                startPressed = true;
                break;
            case "finish":
                if (phase != Phase.RUNNING || run == null || !run.canStop) return "This run can't be finished by hand.";
                finishPressed = true;
                break;
            default:
                return "Unknown action " + action;
        }
        notifyAll();
        return null;
    }

    public synchronized void abort() {
        if (current != null || pending != null) {
            abortRequested = true;
            log("Abort requested");
            notifyAll();
        }
    }

    /** Clears a finished or failed step from the page. */
    public synchronized void dismiss() {
        if (current == null && pending == null) {
            phase = Phase.IDLE;
            promptId++;
        }
    }

    // ------------------------------------------------------------------ OpMode thread

    /** Called from the Calibrate OpMode's loop. Runs a requested step to the end. */
    public void runPending() {
        Step s;
        synchronized (this) {
            s = pending;
            if (s == null) return;
            pending = null;
            current = s;
            abortRequested = false;
            log("Step started: " + s.title);
        }
        try {
            Procedure p = s.factory.get();
            p.setResultPrefix(s.id);
            p.run();
            synchronized (this) {
                phase = Phase.DONE;
                promptId++;
                outcome = results.isEmpty() ? "Finished." : "Finished. Results are saved on the Control Hub.";
                log("Step finished: " + s.title);
            }
        } catch (CalibrationAbort a) {
            fail(a.byUser ? "Stopped: " + a.getMessage() : a.getMessage());
        } catch (InterruptedException e) {
            fail("Stopped: the OpMode was stopped.");
            Thread.currentThread().interrupt();
        } catch (Throwable t) {
            fail("Error: " + t);
        } finally {
            stopMotors();
            synchronized (this) {
                current = null;
                run = null;
                display = null;
                notifyAll();
            }
        }
    }

    private synchronized void fail(String why) {
        phase = Phase.FAILED;
        promptId++;
        outcome = why;
        log(why);
    }

    void awaitInputs(Inputs in) throws InterruptedException {
        synchronized (this) {
            prompt(Phase.INPUTS, in.title, in.message);
            inputs = in;
            answers = null;
            while (answers == null) waitOrAbort();
            for (Inputs.Field<?> f : in.fields) {
                Object v = answers.get(f.name);
                try {
                    f.set(v);
                } catch (RuntimeException e) {
                    throw new CalibrationAbort("Invalid value for " + f.name + ": " + v, false);
                }
            }
            inputs = null;
            log(in.title + ": " + answers);
        }
    }

    void awaitConfirmation(String t, String m) throws InterruptedException {
        synchronized (this) {
            prompt(Phase.CONFIRM, t, m);
            confirmed = false;
            while (!confirmed) waitOrAbort();
        }
    }

    void awaitStart(TuningOpMode<?> op) throws InterruptedException {
        stopMotors(); // nothing moves while the page waits for Start
        synchronized (this) {
            prompt(Phase.READY, op.name, op.description);
            startPressed = false;
            while (!startPressed) waitOrAbort();
            phase = Phase.RUNNING;
            promptId++;
            finishPressed = false;
            samples.clear();
            sampleNames.clear();
            live = new LinkedHashMap<>();
            runStartNanos = System.nanoTime();
            log("Run started: " + op.name);
        }
    }

    <R> R runOpMode(TuningOpMode<R> op) throws InterruptedException {
        Host h;
        synchronized (this) {
            checkAbort();
            run = op;
            h = host;
        }
        try {
            R r = op.execute(h);
            synchronized (this) {
                checkAbort(); // a run that ended because of Abort or Stop doesn't count
                log("Run finished: " + op.name + (r == null ? "" : " → " + r));
            }
            return r;
        } finally {
            stopMotors();
            synchronized (this) {
                run = null;
            }
        }
    }

    synchronized boolean runStopRequested() {
        return finishPressed || abortRequested || host == null || host.stopRequested();
    }

    synchronized void setDisplay(Display d) {
        display = d == null ? null : d.data;
    }

    synchronized void live(Map<String, Object> values) {
        live = values;
        // Plot every number. A new name adds a series; earlier points get NaN for it.
        double t = (System.nanoTime() - runStartNanos) / 1e9;
        List<Double> row = new ArrayList<>();
        for (Map.Entry<String, Object> e : values.entrySet()) {
            if (!(e.getValue() instanceof Number)) continue;
            int i = sampleNames.indexOf(e.getKey());
            if (i < 0) {
                sampleNames.add(e.getKey());
                i = sampleNames.size() - 1;
            }
            while (row.size() <= i) row.add(Double.NaN);
            row.set(i, ((Number) e.getValue()).doubleValue());
        }
        if (row.isEmpty()) return;
        double[] point = new double[sampleNames.size() + 1];
        point[0] = t;
        for (int i = 0; i < sampleNames.size(); i++) point[i + 1] = i < row.size() ? row.get(i) : Double.NaN;
        samples.add(point);
        if (samples.size() > SAMPLES) samples.remove(0);
    }

    synchronized void result(String key, String name, Object value) {
        Map<String, Object> r = new LinkedHashMap<>();
        r.put("name", name);
        r.put("value", value instanceof Enum ? ((Enum<?>) value).name() : value);
        boolean saved = Calibration.isDefined(key);
        if (saved) Calibration.setMeasured(key, value);
        r.put("key", saved ? key : null);
        results.add(r);
    }

    // ------------------------------------------------------------------ page state

    public synchronized Map<String, Object> state() {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("opModeRunning", host != null);
        m.put("phase", phase.name().toLowerCase(Locale.US));
        m.put("promptId", promptId);
        m.put("step", current != null ? current.id : pending != null ? pending.id : null);
        m.put("title", title);
        m.put("message", message);
        m.put("canFinish", phase == Phase.RUNNING && run != null && run.canStop);
        m.put("inputs", inputsDescription());
        m.put("display", display);
        m.put("live", live);
        m.put("sampleNames", new ArrayList<>(sampleNames));
        m.put("samples", new ArrayList<>(samples));
        m.put("results", new ArrayList<>(results));
        m.put("outcome", outcome);
        m.put("log", new ArrayList<>(log));
        m.put("valuesVersion", Calibration.version());
        m.put("file", Calibration.filePath());
        m.put("fileError", Calibration.loadError());
        return m;
    }

    public synchronized List<Object> describeSteps() {
        List<Object> out = new ArrayList<>();
        for (Step s : steps) {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("id", s.id);
            m.put("group", s.group);
            m.put("title", s.title);
            m.put("summary", s.summary);
            List<Object> keys = new ArrayList<>();
            for (String k : s.keys) keys.add(k);
            m.put("keys", keys);
            out.add(m);
        }
        return out;
    }

    // ------------------------------------------------------------------ helpers

    private Object inputsDescription() {
        if (phase != Phase.INPUTS || inputs == null) return null;
        List<Object> fields = new ArrayList<>();
        for (Inputs.Field<?> f : inputs.fields) fields.add(f.describe());
        return fields;
    }

    private void prompt(Phase p, String t, String m) {
        checkAbort();
        phase = p;
        promptId++;
        title = t;
        message = m;
        notifyAll();
    }

    private void waitOrAbort() throws InterruptedException {
        checkAbort();
        wait(50);
        checkAbort();
    }

    private void checkAbort() {
        if (abortRequested) throw new CalibrationAbort("aborted on the web page.", true);
        if (host == null || host.stopRequested()) throw new CalibrationAbort("the OpMode was stopped.", true);
    }

    private void stopMotors() {
        Host h;
        synchronized (this) {
            h = host;
        }
        if (h == null) return;
        try {
            for (DcMotor m : h.hardwareMap().getAll(DcMotor.class)) m.setPower(0);
        } catch (RuntimeException ignored) {
            // the hardware map may already be closed when the OpMode stops
        }
    }

    private Step find(String id) {
        for (Step s : steps) if (s.id.equals(id)) return s;
        return null;
    }

    private void log(String line) {
        log.addLast(new SimpleDateFormat("HH:mm:ss", Locale.US).format(new Date()) + "  " + line);
        while (log.size() > LOG_LINES) log.removeFirst();
    }
}
