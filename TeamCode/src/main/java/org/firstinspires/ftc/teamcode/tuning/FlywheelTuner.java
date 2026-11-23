package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/*
 * Flywheel tuner. All values are live-editable from FTC Dashboard (Config tab).
 *
 * Cross (A):        flywheel on/off
 * D-pad up/down:    TARGET +/- 50
 * Hold R2:          gated feed (same gating as AutoHelper: low + high limit)
 * Square (X):       stagiu full power, no gating (comparison)
 * Hold Circle (B):  kF CALIBRATION - full raw power, read "CAL kF" after ~2 s
 * Triangle (Y):     reset peak / low + clear shot log
 * D-pad left:       mark LAST shot as MISS (press again to undo)
 *
 * Every feed start logs the flywheel speed (% of target). Shots count as HIT
 * unless you mark them. Telemetry shows avg/min/max feed % for hits vs misses.
 *
 * Dashboard Graph: plot "vel" and "target".
 */
@Config
@TeleOp(name = "ShootSimple", group = "tuning")
public class FlywheelTuner extends OpMode {
    public static double KP = 20, KI = 0.2, KD = 0, KF = 15.6;
    public static double TARGET = 1200;                       // ticks/s

    public static double READY_BAND = 0.01;   // enter "at speed" only within +/-1% of target
    public static double SHOT_DIP = 0.025;    // below 97.5% = ball fired -> stop stagiu
    public static double OVER_BAND = 0.02;    // above 102% = too fast -> stop stagiu

    public static double CAL_SETTLE_S = 1.5;  // ignore spin-up before averaging
    public static int LOG_SHOW = 15;          // shots shown in telemetry

    private DcMotorEx turelaD, turelaS;
    private DcMotor stagiu;
    private VoltageSensor battery;

    private boolean on, atSpeed, cal, pFeed;
    private boolean pA, pUp, pDown, pLeft;
    private double aKP = Double.NaN, aKI, aKD, aKF;   // values currently on the hub

    private final ElapsedTime calTimer = new ElapsedTime();
    private double calSum = 0;
    private int calN = 0;

    private double peak = 0, low = Double.MAX_VALUE;

    // shot log: speed at feed start, in % of target
    private final List<Double> shotPct = new ArrayList<>();
    private final List<Boolean> shotMiss = new ArrayList<>();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turelaD = hardwareMap.get(DcMotorEx.class, "turelaD");
        turelaS = hardwareMap.get(DcMotorEx.class, "turelaS");
        turelaS.setDirection(DcMotorSimple.Direction.REVERSE);
        stagiu = hardwareMap.dcMotor.get("stagiu");
        stagiu.setDirection(DcMotorSimple.Direction.REVERSE);
        stagiu.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        battery = hardwareMap.voltageSensor.iterator().next();

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        applyPIDF();
    }

    @Override
    public void loop() {
        // ---------- buttons ----------
        if (gamepad1.a && !pA) on = !on;
        if (gamepad1.dpad_up && !pUp) TARGET += 50;
        if (gamepad1.dpad_down && !pDown) TARGET -= 50;
        if (gamepad1.dpad_left && !pLeft && !shotMiss.isEmpty()) {
            int last = shotMiss.size() - 1;
            shotMiss.set(last, !shotMiss.get(last));          // toggle miss on newest shot
        }
        pA = gamepad1.a; pUp = gamepad1.dpad_up; pDown = gamepad1.dpad_down; pLeft = gamepad1.dpad_left;
        if (gamepad1.y) {
            peak = 0;
            low = Double.MAX_VALUE;
            shotPct.clear();
            shotMiss.clear();
        }

        // ---------- kF calibration ----------
        if (gamepad1.b != cal) {
            cal = gamepad1.b;
            setMode(cal ? DcMotor.RunMode.RUN_WITHOUT_ENCODER : DcMotor.RunMode.RUN_USING_ENCODER);
            if (cal) { calTimer.reset(); calSum = 0; calN = 0; }
            else aKP = Double.NaN;   // force PIDF re-apply after mode change
        }
        if (cal) {
            stagiu.setPower(0);
            turelaD.setPower(1);
            turelaS.setPower(1);
            double vm = avgVel();
            if (calTimer.seconds() > CAL_SETTLE_S) { calSum += vm; calN++; }
            double avg = calN > 0 ? calSum / calN : vm;
            telemetry.addLine("CALIBRATING - hold Circle until avg is stable");
            telemetry.addData("vel now", "%.0f", vm);
            telemetry.addData("CAL max tps (avg)", "%.0f  (%d samples)", avg, calN);
            telemetry.addData("CAL kF = 32767 / max", "%.2f", 32767.0 / avg);
            telemetry.addData("battery V", "%.2f", battery.getVoltage());
            telemetry.update();
            return;
        }

        // ---------- velocity control ----------
        applyPIDF();   // only writes to the hub when a value changed
        turelaD.setVelocity(on ? TARGET : 0);
        turelaS.setVelocity(on ? TARGET : 0);
        double v = avgVel();

        // ---------- gating (identical to AutoHelper) ----------
        double lo = TARGET * (1 - SHOT_DIP), hi = TARGET * (1 + OVER_BAND);
        if (!on || v < lo || v > hi) atSpeed = false;
        else if (Math.abs(v - TARGET) <= TARGET * READY_BAND) atSpeed = true;

        if (gamepad1.x) stagiu.setPower(1.0);
        else if (gamepad1.right_trigger > 0.5 && atSpeed) stagiu.setPower(1.0);
        else stagiu.setPower(0);

        // ---------- shot log: speed at each feed start ----------
        boolean feeding = stagiu.getPower() > 0;
        if (feeding && !pFeed && on && TARGET > 0) {
            shotPct.add(v / TARGET * 100.0);
            shotMiss.add(false);
        }
        pFeed = feeding;

        if (on) { peak = Math.max(peak, v); low = Math.min(low, v); }

        // ---------- telemetry ----------
        telemetry.addData("vel", v);          // numeric -> graphable
        telemetry.addData("target", TARGET);  // numeric -> graphable
        telemetry.addData("err %", "%.2f", on && TARGET > 0 ? (v - TARGET) / TARGET * 100 : 0);
        telemetry.addData("peak / low %", "%.1f / %.1f",
                on && TARGET > 0 ? peak / TARGET * 100 : 0,
                on && TARGET > 0 && low < Double.MAX_VALUE ? low / TARGET * 100 : 0);
        telemetry.addData("at speed", atSpeed);
        telemetry.addData("stagiu", "%.2f", stagiu.getPower());

        telemetry.addData("shots", "%d  (miss %d)", shotPct.size(), countMiss());
        telemetry.addData("HIT  feed %", stats(false));
        telemetry.addData("MISS feed %", stats(true));
        telemetry.addData("log (newest first, X = miss)", logString());

        telemetry.addData("vel D / S", "%.0f / %.0f  diff %.0f",
                turelaD.getVelocity(), turelaS.getVelocity(),
                turelaD.getVelocity() - turelaS.getVelocity());
        PIDFCoefficients c = turelaD.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("PIDF on hub", "%.2f / %.2f / %.2f / %.2f", c.p, c.i, c.d, c.f);
        telemetry.addData("battery V", "%.2f", battery.getVoltage());
        telemetry.update();
    }

    @Override
    public void stop() {
        stagiu.setPower(0);
        turelaD.setPower(0);
        turelaS.setPower(0);
    }

    // ===================== helpers =====================

    private double avgVel() {
        return (turelaD.getVelocity() + turelaS.getVelocity()) / 2.0;
    }

    private void setMode(DcMotor.RunMode mode) {
        turelaD.setMode(mode);
        turelaS.setMode(mode);
    }

    private void applyPIDF() {
        if (KP == aKP && KI == aKI && KD == aKD && KF == aKF) return;
        for (DcMotorEx m : new DcMotorEx[]{turelaD, turelaS}) {
            m.setVelocityPIDFCoefficients(KP, KI, KD, KF);   // always applied, also when KF = 0
        }
        aKP = KP; aKI = KI; aKD = KD; aKF = KF;
    }

    private int countMiss() {
        int n = 0;
        for (boolean m : shotMiss) if (m) n++;
        return n;
    }

    /** avg / min / max feed % for hits (miss=false) or misses (miss=true). */
    private String stats(boolean miss) {
        double sum = 0, mn = Double.MAX_VALUE, mx = -Double.MAX_VALUE;
        int n = 0;
        for (int i = 0; i < shotPct.size(); i++) {
            if (shotMiss.get(i) != miss) continue;
            double p = shotPct.get(i);
            sum += p; mn = Math.min(mn, p); mx = Math.max(mx, p); n++;
        }
        if (n == 0) return "-";
        return String.format("avg %.1f  min %.1f  max %.1f  (n=%d)", sum / n, mn, mx, n);
    }

    private String logString() {
        StringBuilder sb = new StringBuilder();
        int shown = 0;
        for (int i = shotPct.size() - 1; i >= 0 && shown < LOG_SHOW; i--, shown++) {
            sb.append(String.format("%.1f", shotPct.get(i)));
            if (shotMiss.get(i)) sb.append("X");
            sb.append("  ");
        }
        return sb.length() == 0 ? "-" : sb.toString();
    }
}