package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

/*
 * Robot pose handed from AUTO to TELEOP.
 * Auto: Pozitie.set(pinpoint.getPosition()) every loop, Pozitie.saveFile() in stop().
 * TeleOp init: if (Pozitie.load()) pinpoint.setPosition(...).
 * Static fields survive between OpModes. File backup survives a robot app restart.
 */
public class Pozitie {
    public static double x = 0, y = 0, h = 0;   // mm, mm, rad
    public static long savedAtMs = 0;
    public static boolean valid = false;

    public static double MAX_AGE_S = 300;       // older file = last match / yesterday -> ignore

    private static File file() {
        return AppUtil.getInstance().getSettingsFile("pozitie.txt");
    }

    public static void set(Pose2D p) {
        x = p.getX(DistanceUnit.MM);
        y = p.getY(DistanceUnit.MM);
        h = p.getHeading(AngleUnit.RADIANS);
        savedAtMs = System.currentTimeMillis();
        valid = true;
    }

    /** x, y in mm, heading in DEGREES (AutoHelper getX/getY/getHeading units). */
    public static void setMmDeg(double xMm, double yMm, double hDeg) {
        x = xMm;
        y = yMm;
        h = Math.toRadians(hDeg);
        savedAtMs = System.currentTimeMillis();
        valid = true;
    }

    public static void saveFile() {
        if (!valid) return;
        try {
            ReadWriteFile.writeFile(file(), String.format(Locale.US, "%f %f %f %d", x, y, h, savedAtMs));
        } catch (Exception ignored) { }
    }

    /** true = x, y, h hold a fresh pose (static first, then file). */
    public static boolean load() {
        long now = System.currentTimeMillis();
        if (valid && now - savedAtMs < MAX_AGE_S * 1000) return true;
        try {
            String[] t = ReadWriteFile.readFile(file()).trim().split("\\s+");
            long at = Long.parseLong(t[3]);
            if (now - at > MAX_AGE_S * 1000) return false;
            x = Double.parseDouble(t[0]);
            y = Double.parseDouble(t[1]);
            h = Double.parseDouble(t[2]);
            savedAtMs = at;
            valid = true;
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    /** Call at the start of auto so an old pose never leaks into teleop. */
    public static void clear() {
        valid = false;
        savedAtMs = 0;
    }
}