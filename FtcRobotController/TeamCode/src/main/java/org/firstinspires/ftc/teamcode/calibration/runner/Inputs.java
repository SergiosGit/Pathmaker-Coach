package org.firstinspires.ftc.teamcode.calibration.runner;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** A form shown on the web page. Same calls as Pedro AutoTune's Inputs, so its procedures run unchanged. */
public final class Inputs {

    public enum Type { STRING, INTEGER, DOUBLE, BOOLEAN, CHOICE }

    final String title, message;
    final List<Field<?>> fields = new ArrayList<>();

    Inputs(String title, String message) {
        this.title = title;
        this.message = message;
    }

    public Field<String> s(String name) { return add(new Field<String>(name, Type.STRING, null)); }

    public Field<Integer> i(String name) { return add(new Field<Integer>(name, Type.INTEGER, null)); }

    public Field<Double> d(String name) { return add(new Field<Double>(name, Type.DOUBLE, null)); }

    public Field<Boolean> b(String name) { return add(new Field<Boolean>(name, Type.BOOLEAN, null)); }

    public <E extends Enum<E>> Field<E> e(String name, Class<E> type) { return add(new Field<E>(name, Type.CHOICE, type)); }

    private <T> Field<T> add(Field<T> f) {
        fields.add(f);
        return f;
    }

    public static final class Field<T> {
        final String name;
        final Type type;
        final Class<?> enumType;
        T defaultValue;
        T value;

        Field(String name, Type type, Class<?> enumType) {
            this.name = name;
            this.type = type;
            this.enumType = enumType;
        }

        public Field<T> withDefault(T v) {
            defaultValue = v;
            return this;
        }

        public T get() {
            return value != null ? value : defaultValue;
        }

        Map<String, Object> describe() {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("name", name);
            m.put("type", type.name().toLowerCase());
            Object def = defaultValue instanceof Enum ? ((Enum<?>) defaultValue).name() : defaultValue;
            m.put("default", def);
            if (enumType != null) {
                List<Object> options = new ArrayList<>();
                for (Object c : enumType.getEnumConstants()) {
                    Map<String, Object> o = new LinkedHashMap<>();
                    String id = ((Enum<?>) c).name();
                    o.put("id", id);
                    o.put("label", label(enumType, id));
                    options.add(o);
                }
                m.put("options", options);
            }
            return m;
        }

        @SuppressWarnings({"unchecked", "rawtypes"})
        void set(Object raw) {
            if (raw == null) return;
            String s = String.valueOf(raw);
            switch (type) {
                case STRING: value = (T) s; break;
                case INTEGER: value = (T) Integer.valueOf((int) Math.round(Double.parseDouble(s))); break;
                case DOUBLE: value = (T) Double.valueOf(Double.parseDouble(s)); break;
                case BOOLEAN: value = (T) Boolean.valueOf(raw instanceof Boolean ? (Boolean) raw : Boolean.parseBoolean(s)); break;
                case CHOICE: value = (T) Enum.valueOf((Class) enumType, s); break;
            }
        }

        private static String label(Class<?> enumType, String constant) {
            try {
                DisplayName dn = enumType.getField(constant).getAnnotation(DisplayName.class);
                if (dn != null) return dn.value();
            } catch (NoSuchFieldException ignored) { }
            return constant;
        }
    }
}
