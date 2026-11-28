package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.StringReader;
import java.io.StringWriter;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Properties;

/*
 * ALL calibrated robot values in one file on the robot: robot_calib.txt (key=value).
 * "ROBOT CALIBRATION" measures and saves them. Teleop and autos call Calib.load() in init.
 * Every public static (non-final) double below is saved and loaded by name.
 * The values below are the defaults, used until the calibration has run.
 */
@Config
public class Calib {
    // ---- flywheel: power = (KS + KV * speed) * 12 / volts + KP * error ----
    public static double KS = 0.05, KV = 0.00043, KP = 0.003;
    public static double SHOT_SPEED = 1120;             // ticks/s, one fixed speed for every shot

    // ---- turret pot volts (README). FRONT, RIGHT, BACK also set the volts -> angle math ----
    public static double V_FRONT = 0.663, V_RIGHT = 1.030, V_BACK = 1.500;
    public static double V_LEFT = 0.520;                // a target only (full 90 deg left is not reachable)

    // ---- battery model: volts = Voc - BAT_R * amps ----
    public static double BAT_R = 0.06;                  // ohm, battery + switch + wires
    public static double I_DRIVE_FULL = 30;             // amps at 12 V, peak, both drive sides 0 -> full power from stop
    public static double DRIVE_TAU = 0.15;              // s, drive wheels reach ~63 % speed (current decay)
    public static double I_FLY_START = 15;              // amps at 12 V, flywheel 0 -> full power from stop (first 0.3 s)

    // ---- camera on the turret ----
    public static double CAM_PITCH_DEG = 25;            // lens up from horizontal
    public static double DIST_SCALE = 1.0;              // multiplies the Limelight range

    // ---- fixed limits (not calibrated, not saved): software stops inside the 0.46 / 1.79 hard stops ----
    public static final double V_LIMIT_MIN = 0.5, V_LIMIT_MAX = 1.75;

    private static final String FILE = "robot_calib.txt", OLD_FLYWHEEL_FILE = "flywheel.txt";
    public static boolean fromFile = false;             // true = values came from robot_calib.txt

    private static File file(String name) { return AppUtil.getInstance().getSettingsFile(name); }

    private static boolean saved(Field f) {
        int m = f.getModifiers();
        return f.getType() == double.class && Modifier.isStatic(m) && !Modifier.isFinal(m);
    }

    /** Read robot_calib.txt. Missing file: import KS/KV/KP from the old flywheel.txt. Returns true if found. */
    public static boolean load() {
        try {
            String text = ReadWriteFile.readFile(file(FILE));
            if (text == null || text.trim().isEmpty()) throw new RuntimeException("no file");
            Properties p = new Properties();
            p.load(new StringReader(text));
            for (Field f : Calib.class.getFields()) {
                String v = p.getProperty(f.getName());
                if (saved(f) && v != null) f.setDouble(null, Double.parseDouble(v.trim()));
            }
            return fromFile = true;
        } catch (Exception e) {
            importOldFlywheel();
            return fromFile = false;
        }
    }

    /** Write every calibrated value to robot_calib.txt. */
    public static void save() {
        try {
            Properties p = new Properties();
            for (Field f : Calib.class.getFields()) if (saved(f)) p.setProperty(f.getName(), String.valueOf(f.getDouble(null)));
            StringWriter w = new StringWriter();
            p.store(w, "ROBOT CALIBRATION");
            ReadWriteFile.writeFile(file(FILE), w.toString());
            fromFile = true;
        } catch (Exception ignored) { }
    }

    /** Old Flywheel Tuner file: first line "KS KV KP". */
    private static void importOldFlywheel() {
        try {
            String[] k = ReadWriteFile.readFile(file(OLD_FLYWHEEL_FILE)).trim().split("\n")[0].trim().split("\\s+");
            KS = Double.parseDouble(k[0]);
            KV = Double.parseDouble(k[1]);
            KP = Double.parseDouble(k[2]);
        } catch (Exception ignored) { }
    }

    // ===================== turret: pot volts -> angle =====================

    /** Turret angle in the robot frame: 0 = front, + = left (CCW), -90 = right side, -180 = back. */
    public static double turretDeg(double volts) {
        double p = potAngleDeg(volts), pR = potAngleDeg(V_RIGHT);
        double fromSide = p <= pR ? (p - pR) / (pR - potAngleDeg(V_BACK)) * 90.0     // 0 at RIGHT, -90 at BACK
                                  : (p - pR) / (potAngleDeg(V_FRONT) - pR) * 90.0;   // +90 at FRONT
        return fromSide - 90.0;
    }

    /** REV potentiometer on the hub (not linear): 3.3 V -> 0, 0 V -> 270 pot degrees. */
    public static double potAngleDeg(double volts) {
        if (volts < 1e-3) return 270.0;
        double b = 270.0 * volts + 445.5, c = 120285.0 - 36450.0 * volts;
        return (b - Math.sqrt(b * b - 4.0 * volts * c)) / (2.0 * volts);
    }
}
