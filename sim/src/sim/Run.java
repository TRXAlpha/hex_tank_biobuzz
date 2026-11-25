package sim;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Flywheel;
import org.firstinspires.ftc.teamcode.teleop.TeleopAutoAim;
import org.firstinspires.ftc.teamcode.tuning.FlywheelTuner;
import java.util.function.*;

public class Run {
    static final double LOOP = 0.020, DT = 0.001;
    interface Script { void at(double t, OpMode op); }

    static double fwTarget(OpMode op){
        try { java.lang.reflect.Field f = op.getClass().getDeclaredField("fw"); f.setAccessible(true);
              return ((Flywheel) f.get(op)).getTarget(); } catch (Exception e) { return 0; }
    }
    /** Run an OpMode: init, 0.5 s init_loop, start, loop until tEnd. Script sets gamepad / world each loop. */
    static OpMode run(Supplier<OpMode> mk, double tEnd, Script sc, Consumer<OpMode> each){
        OpMode op = mk.get();
        op.init();
        for (int i = 0; i < 25; i++) { sc.at(World.t, op); op.init_loop(); physics(); }
        op.start();
        double t0 = World.t;
        while (World.t - t0 < tEnd) {
            sc.at(World.t - t0, op);
            World.lastTarget = fwTarget(op);
            op.loop();
            if (each != null) each.accept(op);
            physics();
        }
        op.stop();
        return op;
    }
    static void physics(){ for (int i = 0; i < LOOP/DT; i++) { World.step(DT); World.deliver(); } }

    static String tl(String k){ String v = World.tele.last.get(k); return v == null ? "-" : v; }

    // stats
    static double maxAbs, sum; static int n;
    static void st0(){ maxAbs = 0; sum = 0; n = 0; }
    static void st(double e){ maxAbs = Math.max(maxAbs, Math.abs(e)); sum += Math.abs(e); n++; }

    static TeleopAutoAim aim(){ return new TeleopAutoAim(); }
    static void place(double xCm, double yCm, double headDeg, double potV){
        World.x = xCm*10; World.y = yCm*10; World.h = Math.toRadians(headDeg); World.potV = potV; }

    public static void main(String[] a){
        // reach of the turret
        System.out.printf("Turret reach: V 0.5 -> %.0f deg, V 1.75 -> %.0f deg (0 = front, + = left)%n", World.turretDeg(0.5), World.turretDeg(1.75));

        // ===== 1. Flywheel Tuner calibration =====
        World.reset(); World.VOC = 12.6; World.RBAT = 0.04;
        run(FlywheelTuner::new, 18, (t, op) -> World.pad.triangle = t > 0.2 && t < 0.4, null);
        double ksT = World.FLY_FRIC_V/12, kvT = 1/(World.FLY_VPV*12);
        System.out.printf("%n[1] TUNER CALIBRATION: KS %.4f (true %.4f)  KV %.7f (true %.7f)  -> %s%n",
                Flywheel.KS, ksT, Flywheel.KV, kvT, World.tele.lines);

        // ===== 2. Flywheel holds 1300 on good vs old battery, while driving hard =====
        for (double rb : new double[]{0.04, 0.12, 0.18}) {
            World.reset(); World.RBAT = rb;
            Flywheel.TABLE.clear(); // force FIXED speed path: no table -> speedForDistance 0, so use SPEED_FROM_DISTANCE=false
            TeleopAutoAim.SPEED_FROM_DISTANCE = false;
            final double[] mn = {99, 0, 99, 0}; // minV, maxErr after 1.5s, min speed
            run(Run::aim, 8, (t, op) -> { World.pad.left_stick_y = (float) (Math.sin(t*3) > 0 ? 1 : -1); },
                op -> { mn[0] = Math.min(mn[0], World.vbat); if (World.t > 2) { mn[1] = Math.max(mn[1], Math.abs(World.fly - 1300)/13.0); } });
            System.out.printf("[2] battery R=%.2f ohm, driving full fwd/back: lowest %.2f V, flywheel max error %.1f %% of 1300%n", rb, mn[0], mn[1]);
        }
        TeleopAutoAim.SPEED_FROM_DISTANCE = true;
        Flywheel.load();     // tuner file from step 1 (has calibrated KS/KV + table)
        if (Flywheel.TABLE.isEmpty()) { double[][] mm = {{90,1375},{100,1375},{110,1350},{130,1300},{160,1270},{170,1285}}; for (double[] p : mm) Flywheel.putPoint(p[0], p[1]); }
        System.out.println("    table: " + Flywheel.tableString());

        // ===== 3. Standing, hive 35 deg right of front at 150 cm. Turret starts at front. =====
        World.reset(); World.RBAT = 0.04; World.HX = 0; World.HY = 0; World.ROW = 0;
        double hd = 90 + 35;       // robot heading so the hive is 35 deg to the right
        place(-150*Math.cos(Math.toRadians(90)), -150, hd, 0.663);
        final double[] firstFire = {-1}; World.DEBUG = true;
        run(Run::aim, 8, (t, op) -> {}, op -> { if (firstFire[0] < 0 && World.shots > 0) firstFire[0] = World.t; });
        System.out.printf("%n[3] STATIC 150 cm, hive 35 deg right: aim err %.2f deg, dist screen %s vs true %.0f cm, shots %d (first at %.2f s), speed at shots %s%n",
                World.aimErr(), tl("Tags / dist"), World.trueDistCm(), World.shots, firstFire[0], pct());
        World.DEBUG = false;
        System.out.println("    screen: SHOOT=" + tl("SHOOT") + "  Speed=" + tl("Speed") + "  Turret=" + tl("Turret"));

        // ===== 4. Robot spins in place while tags in view, then stops =====
        World.reset(); place(0, -150, 90, 0.663);
        st0();
        run(Run::aim, 6, (t, op) -> World.pad.right_stick_x = (float) (t > 1.5 && t < 2.3 ? 0.5 : t > 3 && t < 3.8 ? -0.5 : 0),
            op -> { if (World.t > 1.5) st(World.aimErr()); });
        System.out.printf("    aim error at each shot: max %.2f deg%n", World.shotAimMax);
        System.out.printf("[4] SPIN in place (0.5 turn, both ways): max aim err %.1f deg, mean %.2f deg, final %.2f deg, shots %d, worst aim at a shot %.2f deg%n", maxAbs, sum/n, World.aimErr(), World.shots, World.shotAimMax);

        // ===== 5. Camera blocked: drive and turn, odometry only =====
        World.reset(); place(0, -150, 90, 0.663);
        st0();
        run(Run::aim, 6, (t, op) -> {
                World.CAMERA_BLOCKED = t > 1.5;
                World.pad.left_stick_y = (float) (t > 1.6 && t < 2.4 ? 0.6 : 0);
                World.pad.right_stick_x = (float) (t > 2.6 && t < 3.4 ? 0.4 : 0); },
            op -> { if (World.t > 1.6) st(World.aimErr()); });
        System.out.printf("[5] CAMERA BLOCKED, drive + turn on odometry: max aim err %.1f deg, final %.2f deg, dist screen %s vs true %.0f cm, shots %d, worst aim at a shot %.2f deg%n",
                maxAbs, World.aimErr(), tl("Tags / dist"), World.trueDistCm(), World.shots, World.shotAimMax);

        // ===== 6. Drive in from 250 cm to 120 cm: range gate + speed follows table =====
        World.reset(); place(0, -250, 90, 0.663);
        final StringBuilder log = new StringBuilder();
        run(Run::aim, 7, (t, op) -> World.pad.left_stick_y = (float) (World.trueDistCm() > 125 && t > 1 ? 0.25 : 0),
            op -> { if (Math.abs(World.t % 1.0) < 0.02) log.append(String.format("      t=%.0f true %.0f cm | %s | %s | %s%n", World.t, World.trueDistCm(), tl("Tags / dist"), tl("Speed"), tl("SHOOT"))); });
        System.out.printf("[6] DRIVE IN 250 -> 120 cm: shots %d, speed at shots %s, worst aim at a shot %.2f deg%n%s", World.shots, pct(), World.shotAimMax, log);

        // ===== 7. Old battery, shooting continuously at 140 cm =====
        World.reset(); World.RBAT = 0.15; place(0, -140, 90, 0.663);
        final double[] lowV = {99};
        run(Run::aim, 6, (t, op) -> World.pad.left_bumper = true, op -> lowV[0] = Math.min(lowV[0], World.vbat));
        System.out.printf("[7] OLD BATTERY 0.15 ohm, 140 cm: shots %d, speed at shots %s, lowest %.2f V%n", World.shots, pct(), lowV[0]);
        World.RBAT = 0.04;

        // ===== 8. Wrong alliance selected =====
        World.reset(); place(0, -150, 90, 0.663);
        run(Run::aim, 3, (t, op) -> World.pad.dpad_left = t < 0, null);   // init_loop gets t<0? no -> see below
        System.out.printf("[8] BLUE selected, blue tags: shots %d (expect >0)%n", World.shots);
        World.reset(); place(0, -150, 90, 0.663);
        run(Run::aim, 3, (t, op) -> World.pad.dpad_left = true, null);
        System.out.printf("    RED selected, only blue tags visible: shots %d (expect 0), screen %s%n", World.shots, tl("SHOOT"));

        // ===== 9. Sensitivity: physical setup errors =====
        System.out.println("\n[9] WHAT PHYSICAL ERRORS DO (robot at 150 cm true):");
        sens("correct setup", () -> {});
        sens("Limelight tag size 165.1 mm (2x)", () -> World.RANGE_SCALE = 2.0);
        sens("Limelight tag size 101.6 mm", () -> World.RANGE_SCALE = 101.6/82.55);
        sens("camera pitch really 30 (code 25)", () -> World.CAM_PITCH_TRUE = 30);
        sens("camera pitch really 20 (code 25)", () -> World.CAM_PITCH_TRUE = 20);
        sens("tags 1000 mm high (not 700)", () -> World.TAG_H = 1000);
        sens("odometry 3% scale error", () -> World.ODO_SCALE = 1.03);
        close();
    }

    static void close(){
        System.out.println("\n[10] YOUR SHOOTING DISTANCES (camera lens 25 deg up):");
        for (double th : new double[]{700, 1000}) for (double d : new double[]{90, 100, 110, 130, 170}) {
            World.reset(); World.TAG_H = th; place(0, -d, 90 + 15, 0.663);
            run(Run::aim, 4, (t, op) -> {}, null);
            System.out.printf("    tags %4.0f mm high, %3.0f cm: dist screen %-12s speed %-12s aim err %5.2f  shots %2d  speed at shots %s  %s%n",
                th, d, tl("Tags / dist"), tl("Speed"), World.aimErr(), World.shots, pct(), tl("SHOOT"));
        }
        World.TAG_H = 700;
        System.out.println("    If the LENS pointed 65 deg up:");
        World.reset(); World.CAM_PITCH_TRUE = 65; place(0, -100, 90 + 15, 0.663);
        run(Run::aim, 4, (t, op) -> {}, null);
        System.out.printf("    100 cm: tags %s, %s%n", tl("Tags / dist"), tl("SHOOT"));
        World.CAM_PITCH_TRUE = 25;
    }

    static void sens(String name, Runnable setup){
        World.reset(); World.RANGE_SCALE = 1; World.CAM_PITCH_TRUE = 25; World.TAG_H = 700; World.ODO_SCALE = 1;
        setup.run(); place(0, -150, 90 + 20, 0.663);
        run(Run::aim, 4, (t, op) -> {}, null);
        System.out.printf("    %-36s dist screen %-12s aim err %5.2f deg  shots %d  %s%n", name, tl("Tags / dist"), World.aimErr(), World.shots, tl("SHOOT"));
        World.RANGE_SCALE = 1; World.CAM_PITCH_TRUE = 25; World.TAG_H = 700; World.ODO_SCALE = 1;
    }
    static String pct(){ if (World.shotPct.isEmpty()) return "-"; double mn = 999, mx = 0; for (double p : World.shotPct) { mn = Math.min(mn, p); mx = Math.max(mx, p); } return String.format("%.1f-%.1f %%", mn, mx); }
}
