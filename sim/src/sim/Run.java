package sim;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Calib;
import org.firstinspires.ftc.teamcode.Flywheel;
import org.firstinspires.ftc.teamcode.Power;
import org.firstinspires.ftc.teamcode.teleop.TeleopAutoAimBlue;
import org.firstinspires.ftc.teamcode.teleop.TeleopAutoAimRed;
import org.firstinspires.ftc.teamcode.teleop.safe.TeleopSafeBlue;
import org.firstinspires.ftc.teamcode.tuning.RobotCalibration;

import java.util.*;
import java.util.function.*;

/** Runs the REAL calibration + teleop code on the simulated robot (World) and prints checks. */
public class Run {
    static final double LOOP = 0.020, DT = 0.001;
    interface Script { void at(double t, OpMode op); }
    static int fails = 0;

    static OpMode run(Supplier<OpMode> mk, double tEnd, Script sc, Consumer<OpMode> each) {
        OpMode op = mk.get();
        op.init();
        for (int i = 0; i < 25; i++) { sc.at(-1, op); op.init_loop(); physics(); }
        op.start();
        double t0 = World.t;
        run0 = t0;
        while (World.t - t0 < tEnd) {
            sc.at(World.t - t0, op);
            World.lastTarget = Calib.SHOT_SPEED;
            op.loop();
            if (each != null) each.accept(op);
            physics();
        }
        op.stop();
        return op;
    }
    static void physics() { for (int i = 0; i < LOOP / DT; i++) { World.step(DT); World.deliver(); } }
    static String tl(String k) { String v = World.tele.last.get(k); return v == null ? "-" : v; }

    /** Button pressed for one loop at each listed time (s). */
    static boolean at(double t, double... times) { for (double x : times) if (t >= x && t < x + LOOP) return true; return false; }

    static void check(String what, boolean ok) { System.out.printf("      %s %s%n", ok ? "PASS" : "FAIL", what); if (!ok) fails++; }

    static void place(double xCm, double yCm, double headDeg, double potV) {
        World.x = xCm * 10; World.y = yCm * 10; World.h = Math.toRadians(headDeg); World.potV = potV; }

    static void calibDefaults() {
        Calib.KS = 0.05; Calib.KV = 0.00043; Calib.KP = 0.003; Calib.SHOT_SPEED = 1120;
        Calib.V_FRONT = 0.663; Calib.V_RIGHT = 1.030; Calib.V_BACK = 1.500; Calib.V_LEFT = 0.520;
        Calib.BAT_R = 0.06; Calib.I_DRIVE_FULL = 30; Calib.DRIVE_TAU = 0.15; Calib.I_FLY_START = 15; Calib.CAM_PITCH_DEG = 25; Calib.DIST_SCALE = 1.0;
    }

    public static void main(String[] a) {
        // ===================== A. ROBOT CALIBRATION =====================
        System.out.println("=== A. ROBOT CALIBRATION (true robot: battery 0.05 ohm, camera pitch 32, tag size wrong x1.4) ===");
        World.reset(); calibDefaults(); com.qualcomm.robotcore.util.ReadWriteFile.FS.clear();
        World.VOC = 12.8; World.RBAT = 0.05; World.CAM_PITCH_TRUE = 32; World.RANGE_SCALE = 1.4;
        place(0, -100, 90, 0.663);                       // 100 cm from the cell, turret forward
        // menu: sel 0 -> CROSS (KS/KV ~16 s) | DOWN, CROSS (KP) | DOWN x2, CROSS (battery) | DOWN, CROSS, CROSS (drive)
        //       | DOWN x2, CROSS (camera: CROSS at 100 cm, teleport, CROSS at 170 cm)
        final double[] tKp = {-1}, tBat = {-1}, tDrv = {-1}, tCam = {-1}, tTur = {-1};
        final double[] drivePos = new double[2];
        run(RobotCalibration::new, 95, (t, op) -> {
            Gamepad g = World.pad;
            g.cross = at(t, 0.5); g.dpad_down = false; g.dpad_up = false; g.dpad_left = false; g.dpad_right = false; g.left_stick_x = 0;
            String menu = World.tele.lines.toString();
            boolean inMenu = menu.contains("D-pad = choose");
            if (inMenu && tKp[0] < 0 && t > 1) { tKp[0] = t; }
            if (tKp[0] > 0) { g.dpad_down = at(t, tKp[0] + 0.1); g.cross = g.cross || at(t, tKp[0] + 0.3); }
            if (tKp[0] > 0 && inMenu && t > tKp[0] + 1 && tBat[0] < 0) tBat[0] = t;
            if (tBat[0] > 0) { g.dpad_down = g.dpad_down || at(t, tBat[0] + 0.1, tBat[0] + 0.3); g.cross = g.cross || at(t, tBat[0] + 0.5); }
            if (tBat[0] > 0 && inMenu && t > tBat[0] + 1 && tDrv[0] < 0) { tDrv[0] = t; drivePos[0] = World.x; drivePos[1] = World.y; }
            if (tDrv[0] > 0) { g.dpad_down = g.dpad_down || at(t, tDrv[0] + 0.1); g.cross = g.cross || at(t, tDrv[0] + 0.3, tDrv[0] + 0.6); }
            if (tDrv[0] > 0 && inMenu && t > tDrv[0] + 1 && tCam[0] < 0) { tCam[0] = t; place(0, -100, 90, 0.663); World.v = 0; World.omega = 0; }
            if (tCam[0] > 0) {
                g.dpad_down = g.dpad_down || at(t, tCam[0] + 0.1, tCam[0] + 0.3);
                g.cross = g.cross || at(t, tCam[0] + 0.5, tCam[0] + 1.0, tCam[0] + 3.0);
                if (at(t, tCam[0] + 2.5)) place(0, -170, 90, 0.663);
            }
            if (tCam[0] > 0 && inMenu && t > tCam[0] + 4.5 && tTur[0] < 0) tTur[0] = t;
            if (tTur[0] > 0) {                                     // turret step: UP x1 from camera = step 6
                double s = t - tTur[0];
                g.dpad_up = at(t, tTur[0] + 0.1);
                g.cross = g.cross || at(t, tTur[0] + 0.3);
                if (s > 0.5 && s < 1.2) g.left_stick_x = 1;      // move turret toward RIGHT (pot V up)
                g.dpad_right = at(t, tTur[0] + 1.5);              // save RIGHT where it is now
                if (s > 1.7 && s < 3.5) g.left_stick_x = 1;       // keep going toward the back
                g.dpad_up = g.dpad_up || at(t, tTur[0] + 3.7);    // try FRONT at a high V -> must be refused
            }
        }, op -> {
            String l = String.join(" | ", World.tele.lines);
            if ((l.contains("KP saved") || l.contains("KP FAILED")) && kpMsg.length() == 0) kpMsg.append(l);
            if (l.contains("CAMERA SAVED") || l.contains("CAMERA NOT")) { camMsg.setLength(0); camMsg.append(l); }
            if (l.contains("BATTERY: rest") && batMsg.length() == 0) batMsg.append(l);
            if (l.contains("DRIVE: start") && drvMsg.length() == 0) drvMsg.append(l);
        });
        System.out.println("   KP step:      " + after(kpMsg, "KP "));
        System.out.println("   battery step: " + after(batMsg, "BATTERY"));
        System.out.println("   drive step:   " + after(drvMsg, "DRIVE"));
        System.out.println("   camera step:  " + after(camMsg, "CAMERA"));
        String msg = String.join(" | ", World.tele.lines);
        double ksT = World.FLY_FRIC_V / 12, kvT = 1 / (World.FLY_VPV * 12);
        System.out.printf("   KS %.4f (true %.4f)  KV %.7f (true %.7f)  KP %.3f  shot speed %.0f%n", Calib.KS, ksT, Calib.KV, kvT, Calib.KP, Calib.SHOT_SPEED);
        check("KS/KV within 20 % / 2 %", Math.abs(Calib.KS - ksT) < 0.2 * ksT + 0.003 && Math.abs(Calib.KV - kvT) < 0.02 * kvT);
        check("KP step finished and saved", kpMsg.indexOf("KP saved") >= 0);
        System.out.printf("   BAT_R %.0f mOhm (true %.0f + model)  I_FLY_START %.1f A  I_DRIVE_FULL %.1f A%n", Calib.BAT_R * 1000, World.RBAT * 1000, Calib.I_FLY_START, Calib.I_DRIVE_FULL);
        check("battery R within 25 %", Math.abs(Calib.BAT_R - World.RBAT) < 0.25 * World.RBAT);
        double peakT = 2 * 2 * 12.0 / 1.3;                     // both sides, 2 motors each, stall
        System.out.printf("   drive: peak %.1f A (true stall ~%.0f A), tau %.3f s (true 0.12)%n", Calib.I_DRIVE_FULL, peakT, Calib.DRIVE_TAU);
        check("drive peak within 25 % of stall, current fade time 0.08..0.25 s", Math.abs(Calib.I_DRIVE_FULL - peakT) < 0.25 * peakT && Calib.DRIVE_TAU > 0.08 && Calib.DRIVE_TAU < 0.25);
        double p130 = camCheck(130);
        System.out.printf("   camera pitch %.1f (true 32)  scale %.3f (true 1/1.4 = %.3f)  -> reads %.1f cm at 130 cm%n", Calib.CAM_PITCH_DEG, Calib.DIST_SCALE, 1 / 1.4, p130);
        check("camera reads 130 +/- 3 cm after calibration", Math.abs(p130 - 130) < 3);
        System.out.printf("   turret saved F/R/B/L %.3f / %.3f / %.3f / %.3f%n", Calib.V_FRONT, Calib.V_RIGHT, Calib.V_BACK, Calib.V_LEFT);
        check("RIGHT saved where the stick moved it, FRONT refused at a high V", Calib.V_RIGHT > 1.03 && Calib.V_FRONT == 0.663);
        check("file robot_calib.txt written", com.qualcomm.robotcore.util.ReadWriteFile.FS.containsKey("robot_calib.txt"));
        System.out.println("   last screen: " + msg.substring(0, Math.min(400, msg.length())));

        // keep the measured values except the turret test position (saved on purpose at a wrong spot)
        Calib.V_RIGHT = 1.030;
        Calib.save();
        World.CAM_PITCH_TRUE = 32; World.RANGE_SCALE = 1.4;      // same robot from here on

        // ===================== B. TELEOP AUTO-AIM =====================
        System.out.println("\n=== B. TELEOP AUTO-AIM (calibrated values) ===");
        // B1: hive 35 deg right at 150 cm, turret forward. Triangle = auto-fire on.
        World.reset(); World.RBAT = 0.05; place(0, -150, 90 + 35, 0.663);
        final double[] maxTarget = {0}, minTarget = {1e9};
        run(TeleopAutoAimBlue::new, 7, (t, op) -> World.pad.triangle = at(t, 0.2), op -> {
            if (World.t - run0 < 0.1) return;                    // first loop: dt = 0, returns early
            double tg = fwTarget(op); maxTarget[0] = Math.max(maxTarget[0], tg); minTarget[0] = Math.min(minTarget[0], tg); });
        System.out.printf("   B1 auto-fire: shots %d, worst aim at a shot %.2f deg, speed at shots %s, dist %s%n", World.shots, World.shotAimMax, pct(), tl("Tags / dist (cam)"));
        check("auto-fire shoots, aim < 4 deg (opening +/-8.5), speed fixed at SHOT_SPEED", World.shots > 5 && World.shotAimMax < 4 && maxTarget[0] == Calib.SHOT_SPEED && minTarget[0] == Calib.SHOT_SPEED);

        // B2: auto-fire OFF -> no shots; R2 -> shots
        World.reset(); place(0, -150, 90 + 35, 0.663);
        run(TeleopAutoAimBlue::new, 4, (t, op) -> {}, null);
        int noFire = World.shots;
        World.reset(); place(0, -150, 90 + 35, 0.663);
        run(TeleopAutoAimBlue::new, 5, (t, op) -> World.pad.right_trigger = t > 3 ? 1 : 0, null);
        System.out.printf("   B2 auto-fire OFF: shots %d (want 0); R2 held: shots %d, worst aim %.2f%n", noFire, World.shots, World.shotAimMax);
        check("no shots without R2 when auto-fire is OFF; R2 shoots on target", noFire == 0 && World.shots > 0 && World.shotAimMax < 4);

        // B3: touchpad -> FIXED FRONT; auto-fire ON must NOT fire; R2 fires; touchpad again -> AIM
        World.reset(); place(0, -150, 90 + 35, 0.663);
        final double[] potAt = {0}; final int[] shotsFixedAuto = {0}, shotsFixedR2 = {0};
        run(TeleopAutoAimBlue::new, 9, (t, op) -> {
            World.pad.triangle = at(t, 0.2);                       // auto-fire ON
            World.pad.touchpad = at(t, 3.0, 7.0);                  // AIM -> FIXED FRONT ... -> AIM
            World.pad.right_trigger = t > 5.0 && t < 6.5 ? 1 : 0;  // R2 in FIXED
        }, op -> {
            if (Math.abs(World.t - (run0 + 4.9)) < 0.011) { potAt[0] = World.potV; shotsFixedAuto[0] = World.shots; }
            if (Math.abs(World.t - (run0 + 6.9)) < 0.011) shotsFixedR2[0] = World.shots;
        });
        System.out.printf("   B3 FIXED FRONT: pot %.3f (want %.3f), screen %s%n", potAt[0], Calib.V_FRONT, tl("TURRET (touchpad)"));
        System.out.printf("      shots in AIM before 3 s + FIXED auto (3..5 s): %d, after R2 in FIXED: %d, total %d%n", shotsFixedAuto[0], shotsFixedR2[0], World.shots);
        check("touchpad: FIXED goes to FRONT (+/-0.03 V)", Math.abs(potAt[0] - Calib.V_FRONT) < 0.03);
        check("R2 shoots in FIXED", shotsFixedR2[0] > shotsFixedAuto[0]);
        check("touchpad again -> back to AIM", tl("TURRET (touchpad)").startsWith("AIM"));

        // B4: D-pad spots
        String[] names = {"UP/FRONT", "DOWN/BACK", "LEFT", "RIGHT"};
        double[] want = {Calib.V_FRONT, Calib.V_BACK, Calib.V_LEFT, Calib.V_RIGHT};
        for (int k = 0; k < 4; k++) {
            final int kk = k;
            World.reset(); place(0, -150, 90 + 35, 1.03);
            run(TeleopAutoAimBlue::new, 3.5, (t, op) -> {
                World.pad.dpad_up = kk == 0 && at(t, 0.3); World.pad.dpad_down = kk == 1 && at(t, 0.3);
                World.pad.dpad_left = kk == 2 && at(t, 0.3); World.pad.dpad_right = kk == 3 && at(t, 0.3);
            }, null);
            System.out.printf("   B4 D-pad %-9s pot %.3f (want %.3f)  %s%n", names[k], World.potV, want[k], tl("TURRET (touchpad)"));
            check("D-pad " + names[k] + " holds its spot", Math.abs(World.potV - want[k]) < 0.03);
        }

        // B5: camera blocked, drive + turn: odometry keeps the aim
        World.reset(); place(0, -150, 90, 0.663);
        final double[] maxErr = {0};
        run(TeleopAutoAimBlue::new, 6, (t, op) -> {
            World.CAMERA_BLOCKED = t > 1.5;
            World.pad.left_stick_y = (float) (t > 1.6 && t < 2.4 ? 0.6 : 0);
            World.pad.right_stick_x = (float) (t > 2.6 && t < 3.4 ? -0.4 : 0);   // turn left: hive stays on the reachable side
        }, op -> { if (World.t > run0 + 4) maxErr[0] = Math.max(maxErr[0], Math.abs(World.aimErr())); });
        System.out.printf("   B5 camera blocked, drive + turn: aim error after moving %.2f deg%n", maxErr[0]);
        check("odometry aim holds within 4 deg (opening +/-8.5)", maxErr[0] < 4);

        // B6: RED teleop: red tags -> shoots; only blue tags in view -> never shoots
        World.reset(); World.FIRST_TAG = 34; World.OTHER_TAGS = new int[]{30, 38, 42}; place(0, -150, 90 + 35, 0.663);
        run(TeleopAutoAimRed::new, 6, (t, op) -> World.pad.triangle = at(t, 0.2), null);
        int redShots = World.shots;
        World.reset(); World.FIRST_TAG = 38; World.OTHER_TAGS = new int[]{}; place(0, -150, 90 + 35, 0.663);
        run(TeleopAutoAimRed::new, 6, (t, op) -> World.pad.triangle = at(t, 0.2), null);
        System.out.printf("   B6 RED teleop: red cell shots %d (want >0), only blue cell in view shots %d (want 0)%n", redShots, World.shots);
        check("RED teleop aims at red cells only", redShots > 0 && World.shots == 0);
        World.FIRST_TAG = 38; World.OTHER_TAGS = new int[]{30, 34, 42};

        // ===================== P. POT FAILURE =====================
        System.out.println("\n=== P. POT FAILURE (TELEOP AUTO-AIM BLUE) ===");
        for (int mode = 1; mode <= 2; mode++) {
            final int md = mode;
            World.reset(); place(0, -150, 90 + 25, 0.663);
            final double[] errAfter = {0}, jerk = {0}; final double[] lastW = {0};
            run(TeleopAutoAimBlue::new, 8, (t, op) -> { World.POT_MODE = md; World.pad.right_trigger = t > 4 ? 1 : 0; },
                op -> { if (World.t > run0 + 3) { errAfter[0] = Math.max(errAfter[0], Math.abs(World.aimErr()));
                        jerk[0] = Math.max(jerk[0], Math.abs(World.potW - lastW[0])); } lastW[0] = World.potW; });
            System.out.printf("   %s pot, tags visible: aim error after 3 s max %.2f deg, R2 shots %d (worst aim %.2f), turret jerk %.3f V/s per loop, screen: %s%n",
                    md == 1 ? "SPIKING" : "DEAD", errAfter[0], World.shots, World.shotAimMax, jerk[0], tl("Turret aim"));
            check((md == 1 ? "spiking" : "dead") + " pot: camera keeps the aim (< 3 deg) and R2 shoots", errAfter[0] < 3 && World.shots > 0 && World.shotAimMax < 3);
        }
        World.reset(); place(0, -150, 90, 0.663);
        final double[] moved = {0}; final double[] pot0 = {World.potV};
        run(TeleopAutoAimBlue::new, 3, (t, op) -> { World.POT_MODE = 2; World.CAMERA_BLOCKED = true; },
            op -> moved[0] = Math.max(moved[0], Math.abs(World.potV - pot0[0])));
        System.out.printf("   DEAD pot, no tags: turret moved %.3f V (want ~0), screen: %s%n", moved[0], tl("Turret aim"));
        check("dead pot + no tags: turret holds still", moved[0] < 0.02);

        // ===================== S. TELEOP SAFE: the 3 guards =====================
        System.out.println("\n=== S. TELEOP SAFE BLUE ===");
        // S0: normal play (drive, turn, shoot with auto-fire): no guard may trigger
        World.reset(); place(0, -150, 90 + 25, 0.663);
        run(TeleopSafeBlue::new, 12, (t, op) -> {
            World.pad.triangle = at(t, 0.2);
            World.pad.left_stick_y = (float) (t > 3 && t < 4 ? -0.5 : t > 6 && t < 7 ? 0.5 : 0);
            World.pad.right_stick_x = (float) (t > 8 && t < 8.6 ? -0.4 : t > 9.5 && t < 10.1 ? 0.4 : 0); }, null);
        System.out.printf("   S0 normal play: shots %d, SAFETY: %s%n", World.shots, tl("SAFETY"));
        check("normal play: no false guard (SAFETY all OK) and it still shoots", tl("SAFETY").startsWith("all OK") && World.shots > 0);
        // S1: the pot slips 0.08 V (~20 deg) at 4 s, tags in view
        World.reset(); place(0, -150, 90 + 25, 0.663);
        final double[] errAfter = {0};
        run(TeleopSafeBlue::new, 10, (t, op) -> { World.POT_OFFSET = t > 4 ? 0.08 : 0; World.pad.right_trigger = t > 7 ? 1 : 0; },
            op -> { if (World.t > run0 + 6) errAfter[0] = Math.max(errAfter[0], Math.abs(World.aimErr())); });
        System.out.printf("   S1 pot slipped 20 deg: aim error after 6 s %.2f deg, R2 shots %d (worst %.2f), SAFETY: %s, aim: %s%n",
                errAfter[0], World.shots, World.shotAimMax, tl("SAFETY"), tl("Turret aim"));
        check("slipped pot: POT SUSPECT, camera keeps the aim, R2 shoots", tl("SAFETY").contains("POT SUSPECT") && errAfter[0] < 3 && World.shots > 0);
        // S2: turret jams at 3 s while the robot turns (turret wants to move)
        World.reset(); place(0, -150, 90 + 25, 0.663);
        final double[] fullPowerMs = {0}; final String[] seen = {""};
        run(TeleopSafeBlue::new, 7, (t, op) -> { World.TURRET_JAMMED = t > 3; World.pad.right_stick_x = (float) (t > 3 && t < 6 ? -0.4 : 0); },
            op -> { if (World.t > run0 + 3 && Math.abs(World.m("turret").power) >= 0.5) fullPowerMs[0] += LOOP * 1000;
                    if (tl("SAFETY").contains("STUCK")) seen[0] = tl("SAFETY"); });
        System.out.printf("   S2 jammed turret: time at high power %.0f ms of 4000 (want small), screen showed: %s%n", fullPowerMs[0], seen[0]);
        check("jammed turret: STUCK shown, high power only in short bursts", seen[0].contains("STUCK") && fullPowerMs[0] < 1500);
        // S2b: the jam clears at 4 s: the turret must move and aim again
        World.reset(); place(0, -150, 90 + 25, 0.663);
        final double[] errClear = {0};
        run(TeleopSafeBlue::new, 9, (t, op) -> { World.TURRET_JAMMED = t > 1 && t < 4; World.pad.right_stick_x = (float) (t > 1.5 && t < 2.1 ? -0.4 : 0); },
            op -> { if (World.t > run0 + 7.5) errClear[0] = Math.max(errClear[0], Math.abs(World.aimErr())); });
        System.out.printf("   S2b jam clears: aim error at the end %.2f deg, SAFETY: %s%n", errClear[0], tl("SAFETY"));
        check("jam clears: turret aims again (< 3 deg)", errClear[0] < 3);
        // S3: Pinpoint fault / frozen, camera blocked: turret must not swing to a wrong place
        for (int k = 0; k < 2; k++) {
            final int kk = k;
            World.reset(); place(0, -150, 90, 0.663);
            final double[] oMoved = {0}; final double[] oPotAt = {0}; final String[] scr = {""};
            run(TeleopSafeBlue::new, 6, (t, op) -> {
                if (kk == 0) World.ODO_FAULT = t > 2; else World.ODO_FROZEN = t > 2;
                World.CAMERA_BLOCKED = t > 2;
                World.pad.right_stick_x = (float) (t > 2.2 && t < 3.0 ? -0.5 : 0);
                World.pad.left_stick_y = (float) (kk == 1 && t > 3.2 && t < 5 ? -0.6 : 0); },
                op -> { if (Math.abs(World.t - (run0 + 2)) < 0.011) oPotAt[0] = World.potV;
                        if (World.t > run0 + 2) oMoved[0] = Math.max(oMoved[0], Math.abs(World.potV - oPotAt[0]));
                        if (tl("SAFETY").contains("ODOMETRY")) scr[0] = tl("SAFETY") + " / " + tl("Turret aim"); });
            System.out.printf("   S3 odometry %s, camera blocked: turret moved %.3f V after the fault, screen: %s%n", kk == 0 ? "FAULT" : "FROZEN", oMoved[0], scr[0]);
            check("odometry " + (kk == 0 ? "fault" : "frozen") + ": detected, turret holds", scr[0].contains("ODOMETRY") && scr[0].contains("HOLD") && oMoved[0] < 0.03 + (kk == 1 ? 0.06 : 0));
        }

        // S4: R1 freeze: robot turns, turret must not move; R2 still shoots; R1 again -> aims again
        World.reset(); place(0, -150, 90 + 25, 0.663);
        final double[] frozenMove = {0}; final double[] fz0 = {0}; final int[] frozenShots = {0}; final double[] errEnd = {0};
        run(TeleopSafeBlue::new, 10, (t, op) -> {
            World.pad.right_bumper = (t > 3 && t < 3.1) || (t > 7 && t < 7.1);
            World.pad.right_stick_x = (float) (t > 4 && t < 4.6 ? -0.5 : 0);
            World.pad.right_trigger = t > 5 && t < 6.5 ? 1 : 0; },
            op -> { double s2 = World.t - run0;
                    if (Math.abs(s2 - 3.2) < 0.011) fz0[0] = World.potV;
                    if (s2 > 3.2 && s2 < 7) frozenMove[0] = Math.max(frozenMove[0], Math.abs(World.potV - fz0[0]));
                    if (Math.abs(s2 - 6.9) < 0.011) frozenShots[0] = World.shots;
                    if (s2 > 9.5) errEnd[0] = Math.max(errEnd[0], Math.abs(World.aimErr())); });
        System.out.printf("   S4 R1 freeze: turret moved %.3f V while frozen (robot turned), R2 shots while frozen %d, aim error after unfreeze %.2f deg%n",
                frozenMove[0], frozenShots[0], errEnd[0]);
        check("R1 freeze: turret does not move, R2 shoots, R1 again aims", frozenMove[0] < 0.005 && frozenShots[0] > 0 && errEnd[0] < 3);

        // burst speed: R2 held 3 s on target, spin-up threshold 90 % vs 95 %
        for (double sd : new double[]{0.90, 0.95}) {
            Flywheel.SPUN_DOWN = sd;
            World.reset(); place(0, -150, 90, 0.663);
            run(TeleopSafeBlue::new, 6, (t, op) -> World.pad.right_trigger = t > 3 ? 1 : 0, null);
            System.out.printf("   burst: SPUN_DOWN %.2f -> %d balls in 3 s, speed at shots %s%n", sd, World.shots, pct());
        }
        Flywheel.SPUN_DOWN = 0.95;

        // ===================== C. BATTERY LIMITER =====================
        System.out.println("\n=== C. BATTERY LIMITER (driver slams full forward / back every 0.4 s, flywheel on) ===");
        // calibrated with the GOOD battery (file says 0.05 ohm); then the weak battery is plugged in
        Calib.BAT_R = 0.05; Calib.save();
        for (double rb : new double[]{0.05, 0.20}) {
            for (boolean on : new boolean[]{false, true}) {
                World.reset(); World.RBAT = rb; place(0, -150, 90, 0.663);
                Power.V_FLOOR = on ? 9.0 : 0.0;
                final double[] lowV = {99}, maxCut = {0}; final double[] dist = {0}; final int[] cutLoops = {0}, loops = {0};
                run(TeleopAutoAimBlue::new, 6, (t, op) -> World.pad.left_stick_y = (float) (t > 1 ? (((int) (t / 0.4)) % 2 == 0 ? 1 : -1) : 0),
                    op -> { learnedR = ohmsOf(op); if (World.t > run0 + 1) { lowV[0] = Math.min(lowV[0], World.vbat); maxCut[0] = Math.max(maxCut[0], cutOf(op)); loops[0]++; if (cutOf(op) > 0.05) cutLoops[0]++; dist[0] += Math.abs(World.v) * LOOP; } });
                System.out.printf("   battery %.2f ohm (file says 0.05), limiter %-3s: lowest %.2f V, max cut %3.0f %% (%d of %d loops), driven %.0f cm, learned R %.0f mOhm%n",
                        rb, on ? "ON" : "OFF", lowV[0], maxCut[0] * 100, cutLoops[0], loops[0], dist[0] / 10, learnedR * 1000);
                if (!on) offDist = dist[0];
                if (on && rb == 0.05) check("good battery: no lag (same distance, brief cuts in < 10 % of loops)", Math.abs(dist[0] - offDist) < 0.02 * offDist && cutLoops[0] < 0.10 * loops[0]);
                if (on && rb == 0.20) check("weak battery: stays above ~8.7 V", lowV[0] > 8.7);
            }
        }
        Power.V_FLOOR = 9.0;

        // ===================== D. AUTOS (real OpModes) =====================
        System.out.println("\n=== D. AUTOS ===");
        String[][] autos = {{"parcare RED", "34"}, {"parcare BLUE", "38"}, {"tilt jos RED", "30"}, {"tilt jos BLUE", "42"}};
        List<Supplier<OpMode>> mk = Arrays.asList(org.firstinspires.ftc.teamcode.auto.red.AutoTrasParcareRed::new,
                org.firstinspires.ftc.teamcode.auto.blue.AutoTrasParcareBlue::new,
                org.firstinspires.ftc.teamcode.auto.red.AutoTrasTiltJosRed::new,
                org.firstinspires.ftc.teamcode.auto.blue.AutoTrasTiltJosBlue::new);
        for (int k = 0; k < 4; k++) {
            int tag = Integer.parseInt(autos[k][1]);
            World.reset(); World.RBAT = 0.05; World.FIRST_TAG = tag;
            List<Integer> others = new ArrayList<>(Arrays.asList(30, 34, 38, 42)); others.remove(Integer.valueOf(tag));
            World.OTHER_TAGS = others.stream().mapToInt(Integer::intValue).toArray();
            if (k < 2) place(0, -150, 180, 1.03); else place(0, -150, -90, 1.50);   // parcare: hive RIGHT, tilt: BACK to the hive
            final double[] flyMax = {0};
            run(mk.get(k), 9, (t, op) -> {}, op -> flyMax[0] = Math.max(flyMax[0], World.fly));
            System.out.printf("   D %-13s (tag %d): flywheel max %.0f, shots %d, speed at shots %s, worst aim %.2f deg, state %s%n",
                    autos[k][0], tag, flyMax[0], World.shots, pct(), World.shotAimMax, tl("state"));
            check(autos[k][0] + ": shoots at SHOT_SPEED on its own cell, then moves on", World.shots > 0 && minPct() > 98.5
                    && World.shotAimMax < 4 && !tl("state").equals("SHOOT") && !tl("state").equals("WAIT_TAGS"));
        }
        // D3: tilt jos with the camera blocked: must NOT shoot (feeds only on live tags), then parks after the timeout
        World.reset(); World.FIRST_TAG = 30; place(0, -150, -90, 1.50); World.CAMERA_BLOCKED = true;
        run(org.firstinspires.ftc.teamcode.auto.red.AutoTrasTiltJosRed::new, 4, (t, op) -> World.CAMERA_BLOCKED = true, null);
        System.out.printf("   D3 tilt jos, camera blocked: shots %d (want 0), state %s%n", World.shots, tl("state"));
        check("tilt jos never shoots without live tags", World.shots == 0);

        System.out.println(fails == 0 ? "\nALL CHECKS PASSED" : "\nFAILED CHECKS: " + fails);
    }

    static double run0;   // set by each run (start time)
    static final StringBuilder kpMsg = new StringBuilder(), camMsg = new StringBuilder(), batMsg = new StringBuilder(), drvMsg = new StringBuilder();
    static String after(StringBuilder sb, String key) { int i = sb.indexOf(key); return i < 0 ? "(none)" : sb.substring(i, Math.min(sb.length(), i + 420)); }

    static double fwTarget(OpMode op) {
        try { java.lang.reflect.Field f = op.getClass().getDeclaredField("fw"); f.setAccessible(true); return ((Flywheel) f.get(op)).getTarget(); }
        catch (Exception e) { return -1; }
    }
    static double learnedR, offDist;
    static double ohmsOf(OpMode op) {
        try { java.lang.reflect.Field f = op.getClass().getDeclaredField("power"); f.setAccessible(true); return ((Power) f.get(op)).ohms(); }
        catch (Exception e) { return -1; }
    }
    static double cutOf(OpMode op) {
        try { java.lang.reflect.Field f = op.getClass().getDeclaredField("power"); f.setAccessible(true); return ((Power) f.get(op)).cut(); }
        catch (Exception e) { return -1; }
    }
    static double minPct() { double mn = 999; for (double p : World.shotPct) mn = Math.min(mn, p); return mn; }
    static String pct() {
        if (World.shotPct.isEmpty()) return "-";
        double mn = 999, mx = 0; for (double p : World.shotPct) { mn = Math.min(mn, p); mx = Math.max(mx, p); }
        return String.format("%.1f-%.1f %%", mn, mx);
    }
    /** Camera distance at a given true distance with the calibrated pitch/scale (uses the real CellVision math). */
    static double camCheck(double cm) {
        World.reset(); place(0, -cm, 90, 0.663);
        for (int i = 0; i < 10; i++) physics();
        com.qualcomm.hardware.limelightvision.LLResult r = World.llLatest;
        List<org.firstinspires.ftc.teamcode.CellVision.Tag> t = org.firstinspires.ftc.teamcode.CellVision.tags(r, 30, 34, 38, 42);
        return org.firstinspires.ftc.teamcode.CellVision.distCm(t, Calib.CAM_PITCH_DEG, Calib.DIST_SCALE);
    }
}
