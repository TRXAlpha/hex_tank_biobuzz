package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/*
 * Flywheel (turelaD + turelaS) with BATTERY-COMPENSATED speed control.
 * Use the SAME class in tuner, teleop and auto.
 *
 *   power = (KS + KV * target) * 12 / batteryVolts  +  KP * (target - speed)
 *
 * KS, KV come from the tuner's auto calibration. They are saved in "flywheel.txt"
 * on the robot together with the distance -> speed table. Flywheel.load() reads them.
 *
 * Usage:
 *   Flywheel.load();                         // once, in init
 *   Flywheel fw = new Flywheel(hardwareMap);
 *   fw.setTarget(Flywheel.speedForDistance(cm));
 *   fw.update();                             // every loop
 *   if (fw.isAtSpeed()) ... feed ...
 */
public class Flywheel {
    // ---- saved by the tuner (defaults = rough guess, run calibration!) ----
    public static double KS = 0.05;    // power needed to just start turning (at 12 V)
    public static double KV = 0.00043; // power per tick/s (at 12 V)
    public static double KP = 0.003;   // extra power per tick/s of error

    // distance (cm) -> speed (ticks/s), sorted by distance.
    // Built-in fallback = values measured on the practice field (Nov 2026).
    // The tuner's saved file replaces these when it exists.
    public static final List<double[]> TABLE = new ArrayList<>();
    static {
        double[][] measured = {{90, 1375}, {100, 1375}, {110, 1350}, {130, 1300}, {160, 1270}, {170, 1285}};
        for (double[] p : measured) TABLE.add(p);
    }

    // auto-fire only inside this range (cm). Team shoots 90-110 cm usually, 170 cm max.
    public static double MIN_SHOT_CM = 90;
    public static double MAX_SHOT_CM = 175;

    public static boolean inRange(double cm) { return cm >= MIN_SHOT_CM && cm <= MAX_SHOT_CM; }

    // ---- feed gating ----
    public static double READY_BAND = 0.01;  // "at speed" = within +/-1 %
    public static double SHOT_DIP = 0.025;   // below 97.5 % = ball left / not ready
    public static double OVER_BAND = 0.02;   // above 102 % = too fast

    public static final double NOMINAL_V = 12.0;
    private static final String FILE = "flywheel.txt";

    private final DcMotorEx d, s;
    private final VoltageSensor battery;
    private double target = 0, vel = 0, volts = NOMINAL_V, power = 0;
    private long lastVoltNs = 0;                 // battery is read every 100 ms, not every loop (slow read)
    private boolean on = false, atSpeed = false;

    public Flywheel(HardwareMap hw) {
        d = hw.get(DcMotorEx.class, "turelaD");
        s = hw.get(DcMotorEx.class, "turelaS");
        d.setDirection(DcMotorSimple.Direction.FORWARD);
        s.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotorEx m : new DcMotorEx[]{d, s}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   // we do the control, not the hub
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m.setPower(0);
        }
        battery = hw.voltageSensor.iterator().next();
        volts = battery.getVoltage();
    }

    // ---------------- control ----------------

    public void setTarget(double tps) { target = Math.max(0, tps); }
    public double getTarget() { return target; }
    public void setOn(boolean on) { this.on = on; }
    public boolean isOn() { return on; }

    /** Call once per loop. Reads speed + battery, sets motor power. */
    public void update() {
        read();
        if (!on || target <= 0) {
            power = 0;
            atSpeed = false;
        } else {
            power = (KS + KV * target) * NOMINAL_V / volts + KP * (target - vel);
            power = Math.max(0, Math.min(1, power));
            double lo = target * (1 - SHOT_DIP), hi = target * (1 + OVER_BAND);
            if (vel < lo || vel > hi) atSpeed = false;
            else if (Math.abs(vel - target) <= target * READY_BAND) atSpeed = true;
        }
        d.setPower(power);
        s.setPower(power);
    }

    /** Raw power, no control (calibration only). */
    public void setRawPower(double p) {
        read();
        power = p;
        atSpeed = false;
        d.setPower(p);
        s.setPower(p);
    }

    private void read() {
        vel = (d.getVelocity() + s.getVelocity()) / 2.0;
        long now = System.nanoTime();
        if (now - lastVoltNs < 100_000_000L) return;
        lastVoltNs = now;
        double v = battery.getVoltage();
        if (v > 5) volts += 0.5 * (v - volts);   // smooth the noise, ignore bad reads
    }

    public void stop() {
        on = false;
        power = 0;
        d.setPower(0);
        s.setPower(0);
    }

    public double getVelocity() { return vel; }
    public double getVelD() { return d.getVelocity(); }
    public double getVelS() { return s.getVelocity(); }
    public double getVolts() { return volts; }
    public double getPower() { return power; }
    public boolean isAtSpeed() { return atSpeed; }

    /** Highest speed the battery can hold right now (keeps 10 % for KP). */
    public double maxSpeedNow() {
        return (0.9 * volts / NOMINAL_V - KS) / KV;
    }

    // ---------------- distance table ----------------

    /** Linear interpolation, clamped at the ends. 0 if table is empty. */
    public static double speedForDistance(double cm) {
        int n = TABLE.size();
        if (n == 0) return 0;
        if (cm <= TABLE.get(0)[0]) return TABLE.get(0)[1];
        if (cm >= TABLE.get(n - 1)[0]) return TABLE.get(n - 1)[1];
        for (int i = 1; i < n; i++) {
            double[] a = TABLE.get(i - 1), b = TABLE.get(i);
            if (cm <= b[0]) return a[1] + (cm - a[0]) / (b[0] - a[0]) * (b[1] - a[1]);
        }
        return TABLE.get(n - 1)[1];
    }

    /** Add or replace the point at this distance, keep sorted. */
    public static void putPoint(double cm, double tps) {
        removePoint(cm);
        int i = 0;
        while (i < TABLE.size() && TABLE.get(i)[0] < cm) i++;
        TABLE.add(i, new double[]{cm, tps});
    }

    public static boolean removePoint(double cm) {
        for (int i = 0; i < TABLE.size(); i++) {
            if (Math.abs(TABLE.get(i)[0] - cm) < 0.5) { TABLE.remove(i); return true; }
        }
        return false;
    }

    // ---------------- file ----------------

    private static File file() {
        return AppUtil.getInstance().getSettingsFile(FILE);
    }

    /** File format: line 1 "KS KV KP", then one "cm tps" line per table point. */
    public static void save() {
        StringBuilder sb = new StringBuilder();
        sb.append(String.format(Locale.US, "%.6f %.8f %.6f\n", KS, KV, KP));
        for (double[] p : TABLE) sb.append(String.format(Locale.US, "%.1f %.1f\n", p[0], p[1]));
        try {
            ReadWriteFile.writeFile(file(), sb.toString());
        } catch (Exception ignored) { }
    }

    /** true = file found and read. False = defaults stay. */
    public static boolean load() {
        try {
            String[] lines = ReadWriteFile.readFile(file()).trim().split("\n");
            String[] k = lines[0].trim().split("\\s+");
            double ks = Double.parseDouble(k[0]), kv = Double.parseDouble(k[1]), kp = Double.parseDouble(k[2]);
            List<double[]> t = new ArrayList<>();
            for (int i = 1; i < lines.length; i++) {
                String[] p = lines[i].trim().split("\\s+");
                if (p.length < 2) continue;
                t.add(new double[]{Double.parseDouble(p[0]), Double.parseDouble(p[1])});
            }
            KS = ks; KV = kv; KP = kp;
            TABLE.clear();
            for (double[] p : t) putPoint(p[0], p[1]);
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    /** Table as text, e.g. "120cm=1150  180cm=1320". */
    public static String tableString() {
        if (TABLE.isEmpty()) return "(empty)";
        StringBuilder sb = new StringBuilder();
        for (double[] p : TABLE) sb.append(String.format(Locale.US, "%.0fcm=%.0f  ", p[0], p[1]));
        return sb.toString();
    }
}
