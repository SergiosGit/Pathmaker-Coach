package org.firstinspires.ftc.teamcode.calibration;

import android.content.Context;

import com.qualcomm.ftccommon.FtcEventLoop;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.teamcode.calibration.runner.CalibrationSession;

import java.io.IOException;
import java.io.InputStream;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import fi.iki.elonen.NanoHTTPD;

/**
 * The calibration web page, at http://192.168.43.1:8090 while connected to the robot.
 * Starts with the robot app, so the page and the saved values are always available; steps run
 * only while the Calibrate OpMode is running.
 */
public class CalibrationServer extends NanoHTTPD {

    public static final int PORT = 8090;
    private static CalibrationServer server;

    /** Supplies the page. On the robot it comes from the app's assets. */
    public interface PageSource {
        InputStream open() throws IOException;
    }

    private final PageSource page;

    public CalibrationServer(int port, PageSource page) {
        super(port);
        this.page = page;
    }

    /** Called by the FTC SDK when the robot app starts. */
    @OnCreateEventLoop
    public static synchronized void start(Context context, FtcEventLoop eventLoop) {
        if (server != null && server.isAlive()) return;
        CalibrationSteps.install();
        final Context app = context.getApplicationContext();
        server = new CalibrationServer(PORT, () -> app.getAssets().open("calibration/index.html"));
        try {
            server.start(SOCKET_READ_TIMEOUT, true);
        } catch (IOException e) {
            server = null; // the robot still runs; only the page is unavailable
        }
    }

    private static long longParam(IHTTPSession session, String name, long otherwise) {
        java.util.List<String> v = session.getParameters().get(name);
        if (v == null || v.isEmpty()) return otherwise;
        try {
            return Long.parseLong(v.get(0));
        } catch (NumberFormatException e) {
            return otherwise;
        }
    }

    @Override
    public Response serve(IHTTPSession session) {
        try {
            String uri = session.getUri();
            if (session.getMethod() == Method.GET) {
                switch (uri) {
                    case "/":
                    case "/index.html":
                        return newFixedLengthResponse(Response.Status.OK, "text/html; charset=utf-8", readPage());
                    case "/api/state":
                        return json(CalibrationSession.get().state(
                                longParam(session, "since", 0), (int) longParam(session, "run", -1)));
                    case "/api/steps":
                        return json(CalibrationSession.get().describeSteps());
                    case "/api/values":
                        return json(Calibration.describeAll());
                    case "/api/export/java":
                        return download(Calibration.exportJava(), "text/x-java; charset=utf-8", "RobotCalibration.java");
                    case "/api/export/shooter":
                        return download(Calibration.exportFields("ShooterConfig"), "text/x-java; charset=utf-8", "ShooterConfig-values.java");
                    case "/api/export/json":
                        return download(Calibration.exportJson(), "application/json; charset=utf-8", "calibration.json");
                    default:
                        return newFixedLengthResponse(Response.Status.NOT_FOUND, MIME_PLAINTEXT, "Not found");
                }
            }
            if (session.getMethod() == Method.POST && uri.equals("/api/action")) {
                Map<String, String> files = new HashMap<>();
                session.parseBody(files);
                String body = files.get("postData");
                return json(action(body == null ? new LinkedHashMap<>() : Json.parseObject(body)));
            }
            return newFixedLengthResponse(Response.Status.METHOD_NOT_ALLOWED, MIME_PLAINTEXT, "Not allowed");
        } catch (Exception e) {
            Map<String, Object> err = new LinkedHashMap<>();
            err.put("error", e.toString());
            return newFixedLengthResponse(Response.Status.INTERNAL_ERROR, "application/json", Json.write(err));
        }
    }

    @SuppressWarnings("unchecked")
    static Map<String, Object> action(Map<String, Object> a) {
        CalibrationSession s = CalibrationSession.get();
        String what = String.valueOf(a.get("action"));
        String error = null;
        switch (what) {
            case "run":
                error = s.requestRun(String.valueOf(a.get("step")));
                break;
            case "abort":
                s.abort();
                break;
            case "dismiss":
                s.dismiss();
                break;
            case "revert": {
                String key = String.valueOf(a.get("key"));
                if (Calibration.isDefined(key)) Calibration.clearMeasured(key);
                else error = "Unknown value " + key;
                break;
            }
            case "set": {
                // Typing a value on the page counts as a measurement made by hand.
                String key = String.valueOf(a.get("key"));
                if (!Calibration.isDefined(key)) {
                    error = "Unknown value " + key;
                } else {
                    try {
                        Calibration.setMeasured(key, a.get("value"));
                    } catch (IllegalArgumentException e) {
                        error = "Not a valid value: " + a.get("value")
                                + (e.getMessage() != null && e.getMessage().contains("one of") ? " (" + e.getMessage() + ")" : "");
                    } catch (RuntimeException e) {
                        error = "Not a valid value: " + a.get("value");
                    }
                }
                break;
            }
            case "inputs":
            case "confirm":
            case "start":
            case "finish":
            case "control": {
                Object id = a.get("promptId");
                long promptId = id instanceof Number ? ((Number) id).longValue() : -1;
                Object values = a.get("values");
                error = s.answer(promptId, what, values instanceof Map ? (Map<String, Object>) values : null);
                break;
            }
            default:
                error = "Unknown action " + what;
        }
        Map<String, Object> r = new LinkedHashMap<>();
        r.put("ok", error == null);
        r.put("error", error);
        return r;
    }

    private String readPage() throws IOException {
        try (InputStream in = page.open()) {
            return Calibration.readAll(in);
        }
    }

    private static Response json(Object o) {
        Response r = newFixedLengthResponse(Response.Status.OK, "application/json; charset=utf-8", Json.write(o));
        r.addHeader("Cache-Control", "no-store");
        return r;
    }

    private static Response download(String body, String mime, String filename) {
        Response r = newFixedLengthResponse(Response.Status.OK, mime, body);
        r.addHeader("Content-Disposition", "attachment; filename=\"" + filename + "\"");
        r.addHeader("Cache-Control", "no-store");
        return r;
    }
}
