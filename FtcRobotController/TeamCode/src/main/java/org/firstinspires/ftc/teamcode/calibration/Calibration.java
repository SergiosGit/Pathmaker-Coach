package org.firstinspires.ftc.teamcode.calibration;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Date;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Every calibrated value the robot uses, with three layers. The first layer that has a value wins:
 * <ol>
 *   <li><b>measured</b>: written by a calibration step, saved on the Control Hub in
 *       {@code /sdcard/FIRST/settings/calibration.json} the moment the step finishes;</li>
 *   <li><b>committed</b>: {@link RobotCalibration}, in git. The web page exports it;</li>
 *   <li><b>default</b>: a safe starting value, defined in {@link CalibrationKeys}.</li>
 * </ol>
 * The file survives app restarts and redeploys, so closing the web page never loses data.
 */
public final class Calibration {

    public enum Source { DEFAULT, COMMITTED, MEASURED }

    /** One calibrated value. */
    public static final class Key {
        public final String id, group, label, unit, help;
        public final Object defaultValue;
        /** If > 0, two measurements must agree within this fraction before the value counts as confirmed. */
        public final double confirmTolerance;
        /** Allowed values for a text setting such as a direction, or null for any. */
        String[] choices;

        Key(String id, String group, String label, String unit, Object defaultValue, double confirmTolerance, String help) {
            this.id = id;
            this.group = group;
            this.label = label;
            this.unit = unit;
            this.defaultValue = defaultValue;
            this.confirmTolerance = confirmTolerance;
            this.help = help;
        }
    }

    /** A measured value and when it was measured. */
    static final class Measurement {
        final Object value;
        final String at;

        Measurement(Object value, String at) {
            this.value = value;
            this.at = at;
        }
    }

    private static final int HISTORY = 5;
    private static final Map<String, Key> keys = new LinkedHashMap<>();
    private static final Map<String, Object> committed = new LinkedHashMap<>();
    /** Newest last. The newest entry is the measured value. */
    private static final Map<String, List<Measurement>> measured = new LinkedHashMap<>();
    private static File file = new File(System.getProperty("calibration.file",
            "/sdcard/FIRST/settings/calibration.json"));
    private static int version = 0;
    private static String loadError = null;

    static {
        CalibrationKeys.defineAll();
        RobotCalibration.apply(committed);
        load();
    }

    private Calibration() { }

    // ---- definitions (used by CalibrationKeys) ----

    static void define(String group, String id, String label, String unit, Object defaultValue, String help) {
        define(group, id, label, unit, defaultValue, 0, help);
    }

    static void define(String group, String id, String label, String unit, Object defaultValue,
                       double confirmTolerance, String help) {
        keys.put(id, new Key(id, group, label, unit, defaultValue, confirmTolerance, help));
    }

    /** Restricts a text setting to the given values (for example FORWARD and REVERSE). */
    static void choices(String id, String... allowed) {
        key(id).choices = allowed;
    }

    // ---- reading ----

    public static synchronized Object get(String id) {
        Key k = key(id);
        List<Measurement> h = measured.get(id);
        if (h != null && !h.isEmpty()) return h.get(h.size() - 1).value;
        if (committed.containsKey(id)) return committed.get(id);
        return k.defaultValue;
    }

    public static double getDouble(String id) { return ((Number) get(id)).doubleValue(); }

    public static int getInt(String id) { return ((Number) get(id)).intValue(); }

    public static String getString(String id) { return String.valueOf(get(id)); }

    public static boolean getBoolean(String id) { return (Boolean) get(id); }

    public static <E extends Enum<E>> E getEnum(String id, Class<E> type) {
        return Enum.valueOf(type, getString(id));
    }

    public static synchronized Source source(String id) {
        key(id);
        List<Measurement> h = measured.get(id);
        if (h != null && !h.isEmpty()) return Source.MEASURED;
        if (committed.containsKey(id)) return Source.COMMITTED;
        return Source.DEFAULT;
    }

    /** True when none of the given values is still a default. */
    public static synchronized boolean allCalibrated(String... ids) {
        for (String id : ids) if (source(id) == Source.DEFAULT) return false;
        return true;
    }

    /** True when every key in the group has a committed or measured value. */
    public static synchronized boolean groupCalibrated(String group) {
        for (Key k : keys.values()) if (k.group.equals(group) && source(k.id) == Source.DEFAULT) return false;
        return true;
    }

    /** False if the key needs two agreeing measurements and doesn't have them yet. */
    public static synchronized boolean confirmed(String id) {
        Key k = key(id);
        if (k.confirmTolerance <= 0 || source(id) != Source.MEASURED) return true;
        List<Measurement> h = measured.get(id);
        if (h.size() < 2) return false;
        double a = ((Number) h.get(h.size() - 1).value).doubleValue();
        double b = ((Number) h.get(h.size() - 2).value).doubleValue();
        return Math.abs(a - b) <= k.confirmTolerance * Math.max(Math.abs(a), Math.abs(b));
    }

    public static boolean isDefined(String id) { return keys.containsKey(id); }

    public static Collection<Key> keys() { return keys.values(); }

    public static synchronized int version() { return version; }

    public static String filePath() { return file.getPath(); }

    public static String loadError() { return loadError; }

    // ---- writing ----

    /** Stores a measured value (converted to the key's type) and saves the file. */
    public static synchronized void setMeasured(String id, Object value) {
        Key k = key(id);
        Object v = convert(value, k.defaultValue);
        if (k.choices != null && !java.util.Arrays.asList(k.choices).contains(v)) {
            throw new IllegalArgumentException(v + " is not one of " + java.util.Arrays.toString(k.choices));
        }
        List<Measurement> h = measured.get(id);
        if (h == null) measured.put(id, h = new ArrayList<>());
        h.add(new Measurement(v, now()));
        while (h.size() > HISTORY) h.remove(0);
        version++;
        save();
    }

    /** Forgets the measured value (and its history), so the committed or default value applies again. */
    public static synchronized void clearMeasured(String id) {
        key(id);
        measured.remove(id);
        version++;
        save();
    }

    /** For tests: use another file and reload it. */
    static synchronized void useFile(File f) {
        file = f;
        measured.clear();
        load();
        version++;
    }

    // ---- web page and export ----

    /** All keys with every layer, for the web page. */
    public static synchronized List<Object> describeAll() {
        List<Object> out = new ArrayList<>();
        for (Key k : keys.values()) {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("id", k.id);
            m.put("group", k.group);
            m.put("label", k.label);
            m.put("unit", k.unit);
            m.put("help", k.help);
            m.put("default", k.defaultValue);
            m.put("committed", committed.get(k.id));
            m.put("hasCommitted", committed.containsKey(k.id));
            List<Measurement> h = measured.get(k.id);
            List<Object> hist = new ArrayList<>();
            if (h != null) {
                for (Measurement me : h) {
                    Map<String, Object> e = new LinkedHashMap<>();
                    e.put("value", me.value);
                    e.put("at", me.at);
                    hist.add(e);
                }
            }
            m.put("history", hist);
            m.put("value", get(k.id));
            m.put("source", source(k.id).name().toLowerCase(Locale.US));
            m.put("confirmed", confirmed(k.id));
            m.put("needsConfirm", k.confirmTolerance > 0);
            if (k.choices != null) m.put("choices", java.util.Arrays.asList((Object[]) k.choices));
            out.add(m);
        }
        return out;
    }

    /** Java source for RobotCalibration.java holding every committed or measured value. */
    public static synchronized String exportJava() {
        StringBuilder sb = new StringBuilder();
        sb.append("package org.firstinspires.ftc.teamcode.calibration;\n\n")
          .append("import java.util.Map;\n\n")
          .append("/**\n")
          .append(" * Calibrated values for this robot, committed to git. Generated by the calibration page\n")
          .append(" * (Export → Java) on ").append(now()).append(".\n")
          .append(" * Values measured later on the Control Hub override these; defaults are in CalibrationKeys.\n")
          .append(" */\n")
          .append("public final class RobotCalibration {\n\n")
          .append("    private RobotCalibration() { }\n\n")
          .append("    static void apply(Map<String, Object> v) {\n");
        String group = null;
        for (Key k : keys.values()) {
            if (source(k.id) == Source.DEFAULT) continue;
            if (!k.group.equals(group)) {
                group = k.group;
                sb.append("        // ").append(group).append('\n');
            }
            sb.append("        v.put(").append(Json.write(k.id)).append(", ").append(javaLiteral(get(k.id))).append(");");
            if (source(k.id) == Source.MEASURED) {
                List<Measurement> h = measured.get(k.id);
                sb.append(" // measured ").append(h.get(h.size() - 1).at);
            }
            sb.append('\n');
        }
        sb.append("    }\n}\n");
        return sb.toString();
    }

    /** The calibration file's content. */
    public static synchronized String exportJson() { return Json.write(fileContent()); }

    // ---- internals ----

    private static Key key(String id) {
        Key k = keys.get(id);
        if (k == null) throw new IllegalArgumentException("Unknown calibration key: " + id);
        return k;
    }

    static Object convert(Object value, Object like) {
        if (value instanceof Enum) value = ((Enum<?>) value).name();
        if (like instanceof Double) {
            if (value instanceof Number) return ((Number) value).doubleValue();
            return Double.parseDouble(String.valueOf(value));
        }
        if (like instanceof Integer) {
            if (value instanceof Number) return (int) Math.round(((Number) value).doubleValue());
            return (int) Math.round(Double.parseDouble(String.valueOf(value)));
        }
        if (like instanceof Boolean) {
            if (value instanceof Boolean) return value;
            return Boolean.parseBoolean(String.valueOf(value));
        }
        return String.valueOf(value);
    }

    private static String javaLiteral(Object v) {
        if (v instanceof Double) return String.valueOf(v);
        if (v instanceof Integer) return String.valueOf(v);
        if (v instanceof Boolean) return String.valueOf(v);
        return Json.write(String.valueOf(v));
    }

    private static String now() {
        return new SimpleDateFormat("yyyy-MM-dd HH:mm", Locale.US).format(new Date());
    }

    private static Map<String, Object> fileContent() {
        Map<String, Object> root = new LinkedHashMap<>();
        root.put("format", 1);
        Map<String, Object> values = new LinkedHashMap<>();
        for (Map.Entry<String, List<Measurement>> e : measured.entrySet()) {
            List<Object> hist = new ArrayList<>();
            for (Measurement m : e.getValue()) {
                Map<String, Object> o = new LinkedHashMap<>();
                o.put("value", m.value);
                o.put("at", m.at);
                hist.add(o);
            }
            values.put(e.getKey(), hist);
        }
        root.put("measured", values);
        return root;
    }

    @SuppressWarnings("unchecked")
    private static void load() {
        loadError = null;
        if (!file.exists()) return;
        try {
            String text = readFile(file);
            Map<String, Object> root = Json.parseObject(text);
            Map<String, Object> values = (Map<String, Object>) root.get("measured");
            if (values == null) return;
            for (Map.Entry<String, Object> e : values.entrySet()) {
                Key k = keys.get(e.getKey());
                if (k == null) continue; // a key that no longer exists
                List<Measurement> h = new ArrayList<>();
                for (Object o : (List<Object>) e.getValue()) {
                    Map<String, Object> m = (Map<String, Object>) o;
                    h.add(new Measurement(convert(m.get("value"), k.defaultValue), String.valueOf(m.get("at"))));
                }
                if (!h.isEmpty()) measured.put(k.id, h);
            }
        } catch (Exception ex) {
            // Keep running on committed and default values; the page shows the problem. Keep the
            // unreadable file as .bad so the next save doesn't destroy it.
            File bad = new File(file.getPath() + ".bad");
            boolean kept = (!bad.exists() || bad.delete()) && file.renameTo(bad);
            loadError = "Could not read " + file + ": " + ex.getMessage()
                    + (kept ? ". It was renamed to " + bad.getName() + "." : "");
            measured.clear();
        }
    }

    // java.nio.file is not available on the Control Hub's Android version (API 25).
    static String readFile(File f) throws IOException {
        try (InputStream in = new FileInputStream(f)) {
            return readAll(in);
        }
    }

    static String readAll(InputStream in) throws IOException {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        byte[] buf = new byte[8192];
        int n;
        while ((n = in.read(buf)) > 0) out.write(buf, 0, n);
        return new String(out.toByteArray(), StandardCharsets.UTF_8);
    }

    private static void save() {
        try {
            File dir = file.getParentFile();
            if (dir != null && !dir.exists() && !dir.mkdirs()) throw new IOException("cannot create " + dir);
            // Write a temporary file first so a crash mid-write can't leave a broken file.
            File tmp = new File(file.getPath() + ".tmp");
            try (Writer w = new OutputStreamWriter(new FileOutputStream(tmp), StandardCharsets.UTF_8)) {
                w.write(Json.write(fileContent()));
            }
            if (!tmp.renameTo(file)) {
                if (!file.delete() || !tmp.renameTo(file)) throw new IOException("cannot replace " + file);
            }
            // A load problem stays on the page for the rest of the session, so it gets noticed.
        } catch (IOException ex) {
            loadError = "Could not save " + file + ": " + ex.getMessage();
        }
    }
}
