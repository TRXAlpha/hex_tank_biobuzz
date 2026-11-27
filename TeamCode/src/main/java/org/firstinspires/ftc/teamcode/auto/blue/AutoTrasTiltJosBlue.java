package org.firstinspires.ftc.teamcode.auto.blue;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Calib;
import org.firstinspires.ftc.teamcode.auto.helpere.AutoHelperTilted;

// BLUE: the path values below are a COPY of the RED ones. Tune them on the blue side.
// Aims at tag cluster 42 (BLUE far CELL).
@Config
@Autonomous(name = "auto tras tilt jos BLUE")
public class AutoTrasTiltJosBlue extends OpMode {

    // ---------- start pose ----------
    // true: the robot starts with its BACK to the hive and the turret pointing BACK (the cable ridge on the
    // turret makes the robot too big when the turret faces the intake). The path numbers below are the ones
    // measured with the FRONT to the hive: go() turns them 180 deg, so the robot drives the same field path.
    public static boolean START_BACK_TO_HIVE = true;

    // ---------- timing ----------
    public static double TAG_TIMEOUT_S = 20;   // no tags by then -> park
    public static double FEED_MS = 5000;       // feed window length

    // ---------- normal route ----------
    public static double heading_first = 0;
    public static double x_first = -230;
    public static double y_first = 0;
    public static double heading_final = -90;
    public static double x_final = -410;
    public static double y_final = 1100;

    // ---------- fallback park (SET THESE) ----------
    public static double heading_park = -90;
    public static double x_park = -410;
    public static double y_park = 1100;

    private enum State {WAIT_TAGS, SHOOT, GO_FIRST, GO_FINAL, PARK, DONE}

    private AutoHelperTilted drive;

    /** lap() with the start-pose turn applied (see START_BACK_TO_HIVE). turnHeading = false: keep the heading
     *  as written (GO_FIRST: 0 = keep the start heading, no 180 deg spin). */
    private boolean go(double heading, double x, double y, boolean turnHeading) {
        if (!START_BACK_TO_HIVE) return drive.lap(heading, x, y, telemetry);
        return drive.lap(turnHeading ? AngleUnit.normalizeDegrees(heading + 180) : heading, -x, -y, telemetry);
    }
    private State state = State.WAIT_TAGS;
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime statetimer = new ElapsedTime();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // fromHardwareMap() already calls resetPosAndIMU()
        drive = AutoHelperTilted.fromHardwareMap(hardwareMap, "stanga", "dreapta", "pinpoint", -44.3, -6.6, 42);
        AutoHelperTilted.V_INIT = START_BACK_TO_HIVE ? Calib.V_BACK : Calib.V_FRONT;   // turret toward the hive in INIT
    }

    @Override
    public void init_loop() {
        drive.turretInitLoop(telemetry);
        telemetry.update();
    }

    @Override
    public void start() {
        drive.startTurret();
        drive.resetMove();
        runtime.reset();
        statetimer.reset();
        state = State.WAIT_TAGS;
    }

    @Override
    public void loop() {
        drive.update(telemetry);   // one Pinpoint read + turret + vision

        switch (state) {
            case WAIT_TAGS:
                drive.setFlywheel(AutoHelperTilted.FLY_TARGET);   // pre-spin while waiting
                if (drive.tagsSeen()) {
                    statetimer.reset();
                    state = State.SHOOT;
                } else if (runtime.seconds() >= TAG_TIMEOUT_S) {
                    drive.stopShooter();
                    drive.resetMove();
                    statetimer.reset();
                    state = State.PARK;
                }
                break;

            case SHOOT:
                if (drive.shootFor(FEED_MS, telemetry)) {
                    drive.resetMove();
                    statetimer.reset();
                    state = State.GO_FIRST;
                }
                break;

            case GO_FIRST:
                if (go(heading_first, x_first, y_first, false)) {
                    drive.resetMove();
                    drive.stop();
                    statetimer.reset();
                    state = State.GO_FINAL;
                }
                break;

            case GO_FINAL:
                if (go(heading_final, x_final, y_final, true)) {
                    drive.resetMove();
                    drive.stop();
                    statetimer.reset();
                    state = State.DONE;
                }
                break;

            case PARK:
                if (go(heading_park, x_park, y_park, true)) {
                    drive.resetMove();
                    drive.stop();
                    statetimer.reset();
                    state = State.DONE;
                }
                break;

            case DONE:
                drive.stop();
                break;
        }

        telemetry.addData("start", START_BACK_TO_HIVE ? "BACK to the hive, turret BACK" : "FRONT to the hive, turret FRONT");
        telemetry.addData("state", state);
        telemetry.addData("tags seen / visible", "%b / %b", drive.tagsSeen(), drive.tagsVisible());
        telemetry.addData("runtime", "%.1f", runtime.seconds());
        telemetry.addData("stateTime", "%.1f", statetimer.seconds());
        telemetry.addData("doneXY / doneHeading", "%b / %b", drive.getStatusXY(), drive.getStatusHeading());
        telemetry.addData("x / y (mm)", "%.0f / %.0f", drive.getX(), drive.getY());
        telemetry.addData("h (deg)", "%.1f", drive.getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stopAll();
    }
}