package org.firstinspires.ftc.teamcode.calibration;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Minimal JSON for the calibration file and the web page: objects, arrays, strings, numbers,
 * booleans and null. Kept here so the calibration code has no extra dependency and can be
 * tested on a laptop.
 */
public final class Json {

    private Json() { }

    // ---- writing ----

    public static String write(Object value) {
        StringBuilder sb = new StringBuilder();
        write(sb, value);
        return sb.toString();
    }

    @SuppressWarnings("unchecked")
    private static void write(StringBuilder sb, Object v) {
        if (v == null) {
            sb.append("null");
        } else if (v instanceof String) {
            quote(sb, (String) v);
        } else if (v instanceof Double || v instanceof Float) {
            double d = ((Number) v).doubleValue();
            if (Double.isNaN(d) || Double.isInfinite(d)) sb.append("null");
            else sb.append(d);
        } else if (v instanceof Number || v instanceof Boolean) {
            sb.append(v);
        } else if (v instanceof Map) {
            sb.append('{');
            boolean first = true;
            for (Map.Entry<String, Object> e : ((Map<String, Object>) v).entrySet()) {
                if (!first) sb.append(',');
                first = false;
                quote(sb, e.getKey());
                sb.append(':');
                write(sb, e.getValue());
            }
            sb.append('}');
        } else if (v instanceof Iterable) {
            sb.append('[');
            boolean first = true;
            for (Object o : (Iterable<Object>) v) {
                if (!first) sb.append(',');
                first = false;
                write(sb, o);
            }
            sb.append(']');
        } else if (v instanceof double[]) {
            sb.append('[');
            double[] a = (double[]) v;
            for (int i = 0; i < a.length; i++) {
                if (i > 0) sb.append(',');
                write(sb, a[i]);
            }
            sb.append(']');
        } else {
            quote(sb, v.toString());
        }
    }

    private static void quote(StringBuilder sb, String s) {
        sb.append('"');
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            switch (c) {
                case '"': sb.append("\\\""); break;
                case '\\': sb.append("\\\\"); break;
                case '\n': sb.append("\\n"); break;
                case '\r': sb.append("\\r"); break;
                case '\t': sb.append("\\t"); break;
                default:
                    if (c < 0x20) sb.append(String.format("\\u%04x", (int) c));
                    else sb.append(c);
            }
        }
        sb.append('"');
    }

    // ---- reading ----

    /** Parses JSON. Objects become LinkedHashMap, arrays ArrayList, numbers Double. */
    public static Object parse(String text) {
        Parser p = new Parser(text);
        p.skipWhitespace();
        Object v = p.value();
        p.skipWhitespace();
        if (p.pos != text.length()) throw p.error("trailing characters");
        return v;
    }

    @SuppressWarnings("unchecked")
    public static Map<String, Object> parseObject(String text) {
        Object v = parse(text);
        if (!(v instanceof Map)) throw new IllegalArgumentException("JSON is not an object");
        return (Map<String, Object>) v;
    }

    private static final class Parser {
        final String s;
        int pos;

        Parser(String s) { this.s = s; }

        IllegalArgumentException error(String what) {
            return new IllegalArgumentException("JSON " + what + " at " + pos);
        }

        void skipWhitespace() {
            while (pos < s.length() && Character.isWhitespace(s.charAt(pos))) pos++;
        }

        Object value() {
            if (pos >= s.length()) throw error("unexpected end");
            char c = s.charAt(pos);
            if (c == '{') return object();
            if (c == '[') return array();
            if (c == '"') return string();
            if (s.startsWith("true", pos)) { pos += 4; return Boolean.TRUE; }
            if (s.startsWith("false", pos)) { pos += 5; return Boolean.FALSE; }
            if (s.startsWith("null", pos)) { pos += 4; return null; }
            return number();
        }

        Map<String, Object> object() {
            Map<String, Object> m = new LinkedHashMap<>();
            pos++;
            skipWhitespace();
            if (peek() == '}') { pos++; return m; }
            while (true) {
                skipWhitespace();
                String key = string();
                skipWhitespace();
                expect(':');
                skipWhitespace();
                m.put(key, value());
                skipWhitespace();
                if (peek() == ',') { pos++; continue; }
                expect('}');
                return m;
            }
        }

        List<Object> array() {
            List<Object> a = new ArrayList<>();
            pos++;
            skipWhitespace();
            if (peek() == ']') { pos++; return a; }
            while (true) {
                skipWhitespace();
                a.add(value());
                skipWhitespace();
                if (peek() == ',') { pos++; continue; }
                expect(']');
                return a;
            }
        }

        String string() {
            expect('"');
            StringBuilder sb = new StringBuilder();
            while (true) {
                if (pos >= s.length()) throw error("unterminated string");
                char c = s.charAt(pos++);
                if (c == '"') return sb.toString();
                if (c != '\\') { sb.append(c); continue; }
                char e = s.charAt(pos++);
                switch (e) {
                    case 'n': sb.append('\n'); break;
                    case 'r': sb.append('\r'); break;
                    case 't': sb.append('\t'); break;
                    case 'b': sb.append('\b'); break;
                    case 'f': sb.append('\f'); break;
                    case 'u':
                        sb.append((char) Integer.parseInt(s.substring(pos, pos + 4), 16));
                        pos += 4;
                        break;
                    default: sb.append(e);
                }
            }
        }

        Double number() {
            int start = pos;
            while (pos < s.length() && "+-0123456789.eE".indexOf(s.charAt(pos)) >= 0) pos++;
            if (start == pos) throw error("unexpected character");
            return Double.parseDouble(s.substring(start, pos));
        }

        char peek() {
            if (pos >= s.length()) throw error("unexpected end");
            return s.charAt(pos);
        }

        void expect(char c) {
            if (peek() != c) throw error("expected '" + c + "'");
            pos++;
        }
    }
}
