package sim;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Calib;
import java.util.*;
import java.util.function.Supplier;

/** Runs each REAL auto for 30 s on the simulated field and checks that it completes its path. */
public class Paths {
    static final double ROBOT_HALF = 22;             // cm, half the robot length (approx.)

    static double endX, endY, endH;
    static boolean potStress = false, stiff = false;
    /** facing: "back" = back to the hive, "front" = front to the hive, "right" = hive on the robot's right. */
    static void auto(String name, Supplier<OpMode> mk, int tag, double startDistCm, String facing, double potV, double rbat) {
        World.reset(); World.RBAT = rbat; World.FIRST_TAG = tag;
        List<Integer> others = new ArrayList<>(Arrays.asList(30, 34, 38, 42)); others.remove(Integer.valueOf(tag));
        World.OTHER_TAGS = others.stream().mapToInt(Integer::intValue).toArray();
        // hive (tag cluster center) at the field origin; robot straight below it
        double head = facing.equals("back") ? -90 : facing.equals("front") ? 90 : 180;
        Run.place(0, -startDistCm, head, potV);
        final List<String> states = new ArrayList<>();
        final double[] minHive = {1e9}, ext = {1e9, -1e9, 1e9, -1e9};
        final StringBuilder track = new StringBuilder();
        Run.run(mk, 30, (t, op) -> { if (potStress) World.POT_MODE = 1; if (stiff) World.TURRET_FRIC = 0.30; }, op -> {
            String st = Run.tl("state");
            if (states.isEmpty() || !states.get(states.size() - 1).startsWith(st + " "))
                states.add(String.format("%s @%.1fs", st, World.t - Run.run0));
            double x = World.x / 10, y = World.y / 10;
            minHive[0] = Math.min(minHive[0], Math.hypot(x, y) - ROBOT_HALF);
            ext[0] = Math.min(ext[0], x); ext[1] = Math.max(ext[1], x); ext[2] = Math.min(ext[2], y); ext[3] = Math.max(ext[3], y);
            double s = World.t - Run.run0;
            if (Math.abs(s - Math.round(s)) < 0.011 && Math.round(s) % 3 == 0)
                track.append(String.format(" (%.0f,%.0f,%.0f)", x, y, Math.toDegrees(World.h)));
        });
        String last = states.get(states.size() - 1);
        endX = World.x / 10; endY = World.y / 10; endH = Math.toDegrees(World.h);
        System.out.printf("%n%s  [start %.0f cm, %s, battery %.2f ohm, turn friction %.2f]%n", name, startDistCm,
                facing.equals("back") ? "BACK to the hive" : facing.equals("front") ? "FRONT to the hive" : "hive on the RIGHT", rbat, World.TURN_FRIC);
        System.out.println("   states: " + String.join(" -> ", states));
        System.out.printf("   shots %d (speed %s), closest robot edge to the hive tags %.0f cm, field x %.0f..%.0f  y %.0f..%.0f cm%n",
                World.shots, Run.pct(), minHive[0], ext[0], ext[1], ext[2], ext[3]);
        System.out.println("   track every 3 s (x, y cm, heading):" + track);
        Run.check(name + ": path completed (DONE)", last.startsWith("DONE"));
        if (!potStress) Run.check(name + ": shot before driving", World.shots > 0);
        Run.check(name + ": done inside the 30 s auto", last.startsWith("DONE") && Double.parseDouble(last.replaceAll(".*@([0-9.]+)s", "$1")) < 29);
        Run.check(name + ": never closer than 20 cm to the hive tags", minHive[0] > 20);
    }

    public static void main(String[] a) {
        Calib.SHOT_SPEED = 1120;
        Supplier<OpMode> tiltR = org.firstinspires.ftc.teamcode.auto.red.AutoTrasTiltJosRed::new;
        Supplier<OpMode> tiltB = org.firstinspires.ftc.teamcode.auto.blue.AutoTrasTiltJosBlue::new;
        // reference: the OLD setup (front to the hive, turret front, numbers as measured)
        org.firstinspires.ftc.teamcode.auto.red.AutoTrasTiltJosRed.START_BACK_TO_HIVE = false;
        auto("REFERENCE old setup: tilt jos RED", tiltR, 30, 110, "front", 0.663, 0.05);
        double rx = endX, ry = endY;
        org.firstinspires.ftc.teamcode.auto.red.AutoTrasTiltJosRed.START_BACK_TO_HIVE = true;
        auto("tilt jos RED", tiltR, 30, 110, "back", 1.50, 0.05);
        System.out.printf("   end (%.0f, %.0f) vs old setup end (%.0f, %.0f)%n", endX, endY, rx, ry);
        Run.check("tilt jos RED: same field path as the old setup (end within 10 cm)", Math.hypot(endX - rx, endY - ry) < 10);
        auto("tilt jos BLUE", tiltB, 42, 110, "back", 1.50, 0.05);
        auto("parcare RED", org.firstinspires.ftc.teamcode.auto.red.AutoTrasParcareRed::new, 34, 110, "right", 1.03, 0.05);
        auto("parcare BLUE", org.firstinspires.ftc.teamcode.auto.blue.AutoTrasParcareBlue::new, 38, 110, "right", 1.03, 0.05);
        // stress: weak battery
        auto("STRESS weak battery: tilt jos RED", tiltR, 30, 110, "back", 1.50, 0.15);
        auto("STRESS weak battery: parcare RED", org.firstinspires.ftc.teamcode.auto.red.AutoTrasParcareRed::new, 34, 110, "right", 1.03, 0.15);
        // stress: robot turns more easily than assumed (slippery tiles / lighter robot)
        World.TURN_FRIC = 0.15;
        auto("STRESS easy turning: tilt jos RED", tiltR, 30, 110, "back", 1.50, 0.05);
        auto("STRESS easy turning: parcare RED", org.firstinspires.ftc.teamcode.auto.red.AutoTrasParcareRed::new, 34, 110, "right", 1.03, 0.05);
        World.TURN_FRIC = 0.25;
        // stress: stiff turret in the tilt auto (friction boost must let it aim and shoot)
        World.TURRET_FRIC = 0.30;
        stiff = true;
        auto("STRESS stiff turret: tilt jos RED", tiltR, 30, 110, "back", 1.50, 0.05);
        stiff = false;
        World.TURRET_FRIC = 0.08;
        // stress: pot spiking during the tilt auto (bad contact): must still finish the path
        potStress = true;
        auto("STRESS spiking pot: tilt jos RED", tiltR, 30, 110, "back", 1.50, 0.05);
        potStress = false;
        System.out.println(Run.fails == 0 ? "\nALL PATH CHECKS PASSED" : "\nFAILED PATH CHECKS: " + Run.fails);
    }
}
