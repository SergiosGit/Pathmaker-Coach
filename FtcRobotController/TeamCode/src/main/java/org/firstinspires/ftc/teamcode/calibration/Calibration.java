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
 * Every calibrated value the robot uses, with three layers:
 * <ol>
 *   <li><b>measured</b>: written by a calibration step or typed on the web page, saved on the
 *       Control Hub in {@code /sdcard/FIRST/settings/calibration.json} at once;</li>
 *   <li><b>committed</b>: in git, in {@link RobotCalibration} or, for values bound to a field, in
 *       that field's initial value (the shooter's are in ShooterConfig.java). The page exports both;</li>
 *   <li><b>default</b>: a safe starting value, defined in {@link CalibrationKeys}.</li>
 * </ol>
 * <b>Whichever changed last wins.</b> A measured value beats the committed one, unless the
 * committed value was changed in Java after the measurement: the file remembers the committed
 * values it saw at the last start, and when one is different now (a deploy changed it), the
 * measured value of that key is dropped and the Java value applies. The page lists those changes.
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
    /** Keys whose value is also written into a public static field, e.g. ShooterConfig.kP. */
    private static final Map<String, java.lang.reflect.Field> fields = new LinkedHashMap<>();
    /** Committed values the file saw at the last start; null if the file predates this. */
    private static Map<String, Object> committedSeen = null;
    private static final List<String> javaChanges = new ArrayList<>();
    private static File file = new File(System.getProperty("calibration.file",
            "/sdcard/FIRST/settings/calibration.json"));
    private static int version = 0;
    private static String loadError = null;

    static {
        CalibrationKeys.defineAll();
        RobotCalibration.apply(committed);
        load();
        applyFields();
    }

    /** Makes sure the values are loaded and written into their fields (ShooterConfig). */
    public static void ensureLoaded() { }

    private Calibration() { }

    // ---- definitions (used by CalibrationKeys) ----

    static void define(String group, String id, String label, String unit, Object defaultValue, String help) {
        define(group, id, label, unit, defaultValue, 0, help);
    }

    static void define(String group, String id, String label, String unit, Object defaultValue,
                       double confirmTolerance, String help) {
        keys.put(id, new Key(id, group, label, unit, defaultValue, confirmTolerance, help));
    }

    /**
     * A value that lives in a public static field (for example ShooterConfig.kP). The field's
     * initial value, as written in Java, is the committed value; the field is then kept equal to
     * the current value, so code that reads the field sees what the page shows.
     */
    static void bind(String group, String id, String label, String unit, Object defaultValue,
                     Class<?> owner, String fieldName, String help) {
        define(group, id, label, unit, defaultValue, help);
        try {
            java.lang.reflect.Field f = owner.getField(fieldName);
            fields.put(id, f);
            committed.put(id, convert(f.get(null), defaultValue));
        } catch (ReflectiveOperationException e) {
            throw new IllegalStateException("No public static field " + owner.getSimpleName() + "." + fieldName, e);
        }
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

    /** Values whose Java version changed since the last start and therefore replaced a measured one. */
    public static synchronized List<String> javaChanges() { return new ArrayList<>(javaChanges); }

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
        applyField(id);
    }

    /** Forgets the measured value (and its history), so the committed or default value applies again. */
    public static synchronized void clearMeasured(String id) {
        key(id);
        measured.remove(id);
        version++;
        save();
        applyField(id);
    }

    /** For tests: use another file and reload it. */
    static synchronized void useFile(File f) {
        file = f;
        measured.clear();
        load();
        applyFields();
        version++;
    }

    /** For tests: what a deploy does to a committed value. */
    static synchronized void setCommittedForTest(String id, Object value) {
        committed.put(id, convert(value, key(id).defaultValue));
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
            java.lang.reflect.Field f = fields.get(k.id);
            m.put("home", f == null ? "RobotCalibration.java" : f.getDeclaringClass().getSimpleName() + ".java");
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
            if (source(k.id) == Source.DEFAULT || fields.containsKey(k.id)) continue;
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

    /**
     * Java lines for the fields of one class (e.g. ShooterConfig), with the current values, to
     * paste over the matching lines in that file.
     */
    public static synchronized String exportFields(String className) {
        StringBuilder sb = new StringBuilder();
        sb.append("    // ===== BEGIN calibration page values (").append(now()).append(") =====\n")
          .append("    // Copied from the calibration page (Copy on the page, or /api/export/shooter).\n")
          .append("    // To update: replace everything from BEGIN to END with a new copy.\n");
        String group = null;
        for (Map.Entry<String, java.lang.reflect.Field> e : fields.entrySet()) {
            java.lang.reflect.Field f = e.getValue();
            if (!f.getDeclaringClass().getSimpleName().equals(className)) continue;
            Key k = keys.get(e.getKey());
            if (!k.group.equals(group)) {
                group = k.group;
                sb.append("\n    // ---- ").append(group).append('\n');
            }
            String type = f.getType() == double.class ? "double" : f.getType() == int.class ? "int"
                    : f.getType() == boolean.class ? "boolean" : "String";
            sb.append("    public static ").append(type).append(' ').append(f.getName()).append(" = ")
              .append(javaLiteral(get(k.id))).append(";  // ");
            if (!k.unit.isEmpty()) sb.append(k.unit).append(". ");
            sb.append(k.help).append('\n');
        }
        sb.append("    // ===== END calibration page values =====\n");
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
        // The committed values as of now: at the next start, a difference means Java changed them.
        root.put("committedSeen", new LinkedHashMap<>(committed));
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
            committedSeen = (Map<String, Object>) root.get("committedSeen");
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
            applyJavaChanges();
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

    /** A committed value that changed since the last start (a deploy) is newer than any measurement. */
    private static void applyJavaChanges() {
        javaChanges.clear();
        if (committedSeen == null) { // first start with this format: just remember what Java says
            save();
            return;
        }
        boolean changed = !committedSeen.keySet().equals(committed.keySet());
        for (Map.Entry<String, Object> e : committed.entrySet()) {
            Key k = keys.get(e.getKey());
            Object seen = committedSeen.containsKey(k.id) ? convert(committedSeen.get(k.id), k.defaultValue) : null;
            if (e.getValue().equals(seen)) continue;
            changed = true;
            List<Measurement> h = measured.get(k.id);
            if (h == null || h.isEmpty()) continue;
            Measurement m = h.get(h.size() - 1);
            if (!m.value.equals(e.getValue())) {
                javaChanges.add(k.label + " (" + k.id + "): the Java value " + e.getValue() + " replaced the measured "
                        + m.value + " from " + m.at + ", because the Java value changed since the last start.");
            }
            measured.remove(k.id);
        }
        if (changed) save();
    }

    private static void applyFields() {
        for (String id : fields.keySet()) applyField(id);
    }

    private static void applyField(String id) {
        java.lang.reflect.Field f = fields.get(id);
        if (f == null) return;
        try {
            Object v = get(id);
            if (f.getType() == double.class) f.setDouble(null, ((Number) v).doubleValue());
            else if (f.getType() == int.class) f.setInt(null, ((Number) v).intValue());
            else if (f.getType() == boolean.class) f.setBoolean(null, (Boolean) v);
            else f.set(null, String.valueOf(v));
        } catch (IllegalAccessException e) {
            throw new IllegalStateException(e);
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
