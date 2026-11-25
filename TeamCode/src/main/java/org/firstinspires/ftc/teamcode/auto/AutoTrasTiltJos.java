package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.helpere.AutoHelper;

/**
 * Wait for tags, shoot, drive. RED values below; BLUE mirrors them (y -> -y, heading -> -heading).
 * Run AutoTrasTiltJosRed or AutoTrasTiltJosBlue (this class is shared code, not an OpMode by itself).
 * Uses AutoHelper.setup() with its own values: far CELL, turret forward in INIT,
 * feed only on live tags, flywheel 1200 (the old AutoHelperTilted values).
 */
@Config
public abstract class AutoTrasTiltJos extends OpMode {

    // ---------- values passed to AutoHelper.setup() ----------
    public static int RED_TAG = 30, BLUE_TAG = 42;   // far CELL: red / blue
    public static double V_INIT = 0.663;             // turret forward in INIT
    public static boolean FEED_NEEDS_TAGS = true;    // feed only on live tags
    public static double FLY_TARGET = 1200;          // ticks/s

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

    private AutoHelper drive;

    /** true = blue alliance (mirror the path). */
    protected abstract boolean blue();

    /** lap() with the alliance mirror applied. */
    private boolean go(double heading, double x, double y) {
        return blue() ? drive.lap(-heading, x, -y, telemetry) : drive.lap(heading, x, y, telemetry);
    }
    private State state = State.WAIT_TAGS;
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime statetimer = new ElapsedTime();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // fromHardwareMap() already calls resetPosAndIMU()
        drive = AutoHelper.fromHardwareMap(hardwareMap, "stanga", "dreapta", "pinpoint", -44.3, -6.6);
        drive.setup(blue() ? BLUE_TAG : RED_TAG, V_INIT, FEED_NEEDS_TAGS, FLY_TARGET);
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
                drive.setFlywheel(FLY_TARGET);   // pre-spin while waiting
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
                if (go(heading_first, x_first, y_first)) {
                    drive.resetMove();
                    drive.stop();
                    statetimer.reset();
                    state = State.GO_FINAL;
                }
                break;

            case GO_FINAL:
                if (go(heading_final, x_final, y_final)) {
                    drive.resetMove();
                    drive.stop();
                    statetimer.reset();
                    state = State.DONE;
                }
                break;

            case PARK:
                if (go(heading_park, x_park, y_park)) {
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

        telemetry.addData("alliance", blue() ? "BLUE (mirrored)" : "RED");
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