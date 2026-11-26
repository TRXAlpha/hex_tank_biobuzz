package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Calib;
import org.firstinspires.ftc.teamcode.CellVision;
import org.firstinspires.ftc.teamcode.Flywheel;

import java.util.ArrayList;
import java.util.List;

/*
 * ROBOT CALIBRATION - one OpMode for every physical value. The screen says what to do.
 * D-pad UP/DOWN = pick a step, CROSS = start it, SHARE = back to the menu (stops everything).
 * Every step saves its result to robot_calib.txt by itself. Teleop and autos load that file.
 *
 *   1 FLYWHEEL KS/KV   (auto, ~15 s)  balls out. Power sweep -> feedforward.
 *   2 FLYWHEEL KP      (auto, ~25 s)  balls out. Fake ball dips -> fastest recovery without overshoot.
 *   3 SHOT SPEED       (manual)       shoot, adjust, CIRCLE saves the one fixed speed.
 *   4 BATTERY          (auto, ~6 s)   balls out. Battery + wiring resistance, flywheel start current.
 *   5 DRIVE CURRENT    (auto, 1 s)    ROBOT DRIVES ~40 cm. Drive start current for the battery model.
 *   6 TURRET POSITIONS (manual)       left stick moves the turret, D-pad saves FRONT/RIGHT/BACK/LEFT.
 *   7 CAMERA           (manual)       tape 100 cm + 170 cm -> camera pitch + distance scale.
 */
@Config
@TeleOp(name = "ROBOT CALIBRATION", group = "tuning")
public class RobotCalibration extends OpMode {
    public static int PIPELINE = 2;
    public static double REF_NEAR_CM = 100, REF_FAR_CM = 170;   // camera step: tape distances
    public static double DRIVE_DIR = -1;                        // drive test direction (-1 = stick-up direction in teleop)
    public static double TURRET_STICK_POWER = 0.4;

    private static final String[] STEPS = {"1 FLYWHEEL KS/KV (auto)", "2 FLYWHEEL KP (auto)", "3 SHOT SPEED",
            "4 BATTERY (auto)", "5 DRIVE CURRENT (robot moves!)", "6 TURRET POSITIONS", "7 CAMERA"};
    private static final double[] SWEEP = {0.3, 0.45, 0.6, 0.75, 0.9};
    private static final double[] KP_TRY = {0.001, 0.002, 0.003, 0.005, 0.008};

    private Flywheel fw;
    private DcMotorEx stanga, dreapta, intake, stagiu;
    private DcMotor turret;
    private AnalogInput pot;
    private Limelight3A ll;
    private VoltageSensor bat;

    private int sel = 0, step = -1, phase = 0;
    private final boolean[] done = new boolean[STEPS.length];
    private String msg = "";
    private final ElapsedTime tm = new ElapsedTime();         // time in the current phase
    private double volts = 12, potV = 1;

    // buttons: pressed THIS loop (edge)
    private boolean up, down, left, right, cross, circle, share, l1, r1;
    private final boolean[] prev = new boolean[9];

    // shared step data
    private final double[] a = new double[8], b = new double[8];
    private double sumA, sumB, sumC, sumD, minV, maxI, t0;
    private int n;
    private final List<String> log = new ArrayList<>();
    private double shotSpeed;
    private boolean wasFeeding, wasOn;
    private List<CellVision.Tag> near, far;
    private int camBase = -1;
    private final double[][] tagSum = new double[4][4];       // per tag: range, tx, ty, count
    private int frames;
    private double lastStale = 1e9;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        Calib.load();
        fw = new Flywheel(hardwareMap);
        stanga = hardwareMap.get(DcMotorEx.class, "stanga");
        dreapta = hardwareMap.get(DcMotorEx.class, "dreapta");
        dreapta.setDirection(DcMotorSimple.Direction.REVERSE);          // same as teleop
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        stagiu = hardwareMap.get(DcMotorEx.class, "stagiu");
        stagiu.setDirection(DcMotorSimple.Direction.REVERSE);
        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        for (DcMotor m : new DcMotor[]{stanga, dreapta, intake, stagiu, turret}) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pot = hardwareMap.get(AnalogInput.class, "potTurela");
        bat = hardwareMap.voltageSensor.iterator().next();
        ll = hardwareMap.get(Limelight3A.class, "limelight");
        ll.setPollRateHz(100);
        ll.pipelineSwitch(PIPELINE);
        ll.start();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("ROBOT CALIBRATION. Use the GOOD, fully charged battery. Press START.");
        telemetry.addData("Saved values", Calib.fromFile ? "found (robot_calib.txt)" : "none yet - defaults");
        telemetry.update();
    }

    @Override
    public void loop() {
        double v = bat.getVoltage();
        if (v > 3) volts = v;
        potV = pot.getVoltage();
        readButtons();

        if (step >= 0 && share) { if (step == 1) Calib.KP = oldKp; allOff(); msg = "Cancelled."; step = -1; }
        if (step < 0) { menu(); return; }

        switch (step) {
            case 0: stepSweep(); break;
            case 1: stepKp(); break;
            case 2: stepShotSpeed(); break;
            case 3: stepBattery(); break;
            case 4: stepDrive(); break;
            case 5: stepTurret(); break;
            case 6: stepCamera(); break;
        }
        telemetry.addData("Battery", "%.2f V", volts);
        telemetry.addLine("SHARE = back to menu");
        telemetry.update();
    }

    // ===================== menu =====================

    private void menu() {
        allOff();
        if (up) sel = (sel + STEPS.length - 1) % STEPS.length;
        if (down) sel = (sel + 1) % STEPS.length;
        if (cross) { begin(sel); return; }
        telemetry.addLine("D-pad = choose, CROSS = start");
        for (int i = 0; i < STEPS.length; i++) {
            telemetry.addLine((i == sel ? ">> " : "    ") + STEPS[i] + (done[i] ? "   DONE" : ""));
        }
        if (!msg.isEmpty()) telemetry.addLine("\n" + msg);
        telemetry.addLine("");
        telemetry.addData("Flywheel KS / KV / KP", "%.3f / %.6f / %.4f", Calib.KS, Calib.KV, Calib.KP);
        telemetry.addData("Shot speed", "%.0f", Calib.SHOT_SPEED);
        telemetry.addData("Battery R / drive A, tau / fly A", "%.0f mOhm / %.0f, %.2f s / %.0f", Calib.BAT_R * 1000,
                Calib.I_DRIVE_FULL, Calib.DRIVE_TAU, Calib.I_FLY_START);
        telemetry.addData("Turret F / R / B / L", "%.3f / %.3f / %.3f / %.3f", Calib.V_FRONT, Calib.V_RIGHT, Calib.V_BACK, Calib.V_LEFT);
        telemetry.addData("Camera pitch / scale", "%.1f / %.3f", Calib.CAM_PITCH_DEG, Calib.DIST_SCALE);
        telemetry.addData("Battery", "%.2f V", volts);
        telemetry.update();
    }

    private void begin(int s) {
        step = s;
        phase = 0;
        msg = "";
        n = 0;
        sumA = sumB = sumC = sumD = 0;
        minV = 99;
        maxI = 0;
        log.clear();
        tm.reset();
        shotSpeed = Calib.SHOT_SPEED;
        wasOn = wasFeeding = false;
        near = far = null;
        camBase = -1;
        sub = 0;
        stableSince = 0;
        oldKp = Calib.KP;
    }

    private void finish(int s, String result) {
        allOff();
        done[s] = true;
        msg = result;
        step = -1;
    }

    // ===================== 1: flywheel KS / KV =====================

    private void stepSweep() {
        double p = SWEEP[phase];
        fw.setRawPower(p);
        if (tm.seconds() > 2.0) { sumA += fw.getVelocity(); sumB += volts; n++; }
        if (tm.seconds() > 3.0) {
            a[phase] = sumA / n;                                          // speed
            b[phase] = p * (sumB / n) / Flywheel.NOMINAL_V;               // power as if 12 V
            sumA = sumB = 0;
            n = 0;
            tm.reset();
            if (++phase == SWEEP.length) { fitSweep(); return; }
        }
        telemetry.addLine("FLYWHEEL KS/KV - balls OUT, do not touch.");
        telemetry.addData("Power", "%.2f  (%d / %d)", p, phase + 1, SWEEP.length);
        telemetry.addData("Speed", "%.0f", fw.getVelocity());
    }

    /** Straight line power = KS + KV * speed through the sweep points. */
    private void fitSweep() {
        int k = SWEEP.length;
        double mv = 0, me = 0;
        for (int i = 0; i < k; i++) { mv += a[i]; me += b[i]; }
        mv /= k; me /= k;
        double num = 0, den = 0;
        for (int i = 0; i < k; i++) { num += (a[i] - mv) * (b[i] - me); den += (a[i] - mv) * (a[i] - mv); }
        double kv = den > 0 ? num / den : 0, ks = me - kv * mv;
        if (kv <= 0 || ks < -0.05 || ks > 0.3) {
            finish(0, String.format("KS/KV FAILED (KS %.3f KV %.6f). Flywheel stuck or encoder unplugged? Not saved.", ks, kv));
            done[0] = false;
            return;
        }
        Calib.KS = Math.max(0, ks);
        Calib.KV = kv;
        Calib.save();
        finish(0, String.format("KS/KV saved: %.3f / %.6f. Max speed at 12 V ~%.0f. Next: step 2.", Calib.KS, kv, (1 - ks) / kv));
    }

    // ===================== 2: flywheel KP (fake ball dips) =====================
    // per candidate: spin up (stable 1 s) -> fake ball: BRAKE until the speed fell 5 % (like one ball)
    //                -> measure recovery to +/-1 % and overshoot (1.2 s)

    private int sub;              // 0 spin, 1 dip, 2 recover
    private double stableSince, recT, over, dip, oldKp;
    private final double[] recov = new double[KP_TRY.length], overs = new double[KP_TRY.length];

    private void stepKp() {
        double target = Calib.SHOT_SPEED;
        Calib.KP = KP_TRY[phase];
        fw.setTarget(target);
        fw.setOn(true);
        double err = (fw.getVelocity() - target) / target;
        if (sub == 0) {
            fw.update(volts);
            if (Math.abs(err) > 0.01) stableSince = tm.seconds();
            if (tm.seconds() - stableSince > 1.0) { sub = 1; tm.reset(); }
            else if (tm.seconds() > 6) { kpResult(-1, 0, 0); }                  // never stable
        } else if (sub == 1) {
            fw.setBrake(true);
            fw.setRawPower(0);
            if (err <= -0.05 || tm.seconds() > 0.5) {
                fw.setBrake(false);
                sub = 2; tm.reset(); recT = -1; over = 0; dip = 0; stableSince = -1;
            }
        } else {
            fw.update(volts);
            dip = Math.min(dip, err);
            over = Math.max(over, err);
            if (Math.abs(err) <= 0.01) { if (stableSince < 0) stableSince = tm.seconds(); }
            else stableSince = -1;
            if (recT < 0 && stableSince >= 0 && tm.seconds() - stableSince >= 0.1) recT = stableSince;
            if (tm.seconds() > 1.2) kpResult(recT, over, dip);
        }
        if (step != 1) return;                                // finished: last candidate done
        telemetry.addLine("FLYWHEEL KP - balls OUT, do not touch.");
        telemetry.addData("Trying KP", "%.3f  (%d / %d)  %s", KP_TRY[phase], phase + 1, KP_TRY.length,
                sub == 0 ? "spinning up" : sub == 1 ? "fake ball" : "recovering");
        telemetry.addData("Speed", "%.0f / %.0f", fw.getVelocity(), target);
        for (String s : log) telemetry.addLine(s);
    }

    private void kpResult(double rec, double ov, double dp) {
        recov[phase] = rec;                                   // recovery time (s), -1 = failed
        overs[phase] = ov;                                    // overshoot (fraction)
        log.add(String.format("KP %.3f: dip %.1f%%  back in %s  overshoot %.1f%%", KP_TRY[phase], dp * 100,
                rec < 0 ? "NEVER" : String.format("%.2f s", rec), ov * 100));
        sub = 0;
        stableSince = 0;
        tm.reset();
        if (++phase < KP_TRY.length) return;
        int best = -1;
        for (int i = 0; i < KP_TRY.length; i++)                   // fastest with overshoot <= 1 %
            if (recov[i] >= 0 && overs[i] <= 0.01 && (best < 0 || recov[i] < recov[best])) best = i;
        if (best < 0) for (int i = 0; i < KP_TRY.length; i++)     // else: least overshoot that recovered
            if (recov[i] >= 0 && (best < 0 || overs[i] < overs[best])) best = i;
        if (best < 0) {
            Calib.KP = oldKp;
            finish(1, "KP FAILED: speed never came back. Run step 1 first. KP not changed.");
            done[1] = false;
            return;
        }
        Calib.KP = KP_TRY[best];
        Calib.save();
        StringBuilder sb = new StringBuilder(String.format("KP saved: %.3f (back in %.2f s).", Calib.KP, recov[best]));
        for (String s : log) sb.append("\n").append(s);
        finish(1, sb.toString());
    }

    // ===================== 3: shot speed =====================

    private void stepShotSpeed() {
        if (cross) fw.setOn(!fw.isOn());
        if (up) shotSpeed += 5;
        if (down) shotSpeed -= 5;
        if (r1) shotSpeed += 50;
        if (l1) shotSpeed -= 50;
        shotSpeed = Math.max(0, shotSpeed);
        if (circle) { Calib.SHOT_SPEED = shotSpeed; Calib.save(); done[2] = true; msg = "Saved shot speed " + (int) shotSpeed; }

        // battery model check: flywheel turned on from (near) stop -> predicted vs measured lowest volts
        if (fw.isOn() && !wasOn && fw.getVelocity() < 100) {
            sumA = volts - Calib.BAT_R * Calib.I_FLY_START;          // predicted lowest
            minV = volts;
            t0 = getRuntime();
        }
        if (fw.isOn() && getRuntime() - t0 < 1.0) minV = Math.min(minV, volts);
        wasOn = fw.isOn();

        fw.setTarget(shotSpeed);
        fw.update(volts);
        boolean feed = gamepad1.right_trigger > 0.5 && fw.isOn() && fw.isAtSpeed();
        intake.setPower(feed ? 1 : 0);
        stagiu.setPower(feed ? 1 : 0);
        if (feed && !wasFeeding) {
            log.add(0, String.format("%.1f%%", fw.getVelocity() / shotSpeed * 100));
            if (log.size() > 8) log.remove(8);
        }
        wasFeeding = feed;

        telemetry.addLine("SHOT SPEED - CROSS flywheel on/off, hold R2 shoot.");
        telemetry.addLine("Short: D-pad UP (+5) / R1 (+50). Long: D-pad DOWN (-5) / L1 (-50).");
        telemetry.addLine("Goes in every time -> CIRCLE saves it.");
        if (!msg.isEmpty()) telemetry.addLine(msg);
        telemetry.addData("SPEED", "%.0f   (saved %.0f)", shotSpeed, Calib.SHOT_SPEED);
        telemetry.addData("Flywheel", "%.0f  %s", fw.getVelocity(), fw.isOn() ? (fw.isAtSpeed() ? "READY" : "...") : "OFF");
        telemetry.addData("Shots (speed at feed)", String.join("  ", log));
        if (minV < 99) telemetry.addData("Spin-up lowest V", "measured %.2f  model %.2f", minV, sumA);
        if (fw.isOn() && shotSpeed > fw.maxSpeedNow()) telemetry.addLine("!! TOO FAST for this battery !!");
    }

    // ===================== 4: battery =====================

    private void stepBattery() {
        String what;
        if (phase == 0) {                                     // brake the flywheel to a stop, everything off
            allOff();
            fw.setBrake(true);
            what = "stopping the flywheel";
            if (Math.abs(fw.getVelD()) < 30 || tm.seconds() > 4) { fw.setBrake(false); next(); }
        } else if (phase == 1) {                              // rest volts
            what = "rest";
            sumA += volts; n++;
            if (tm.seconds() > 0.5) { a[0] = sumA / n; sumA = 0; n = 0; next(); }
        } else if (phase == 2) {                              // flywheel full power from stop
            what = "flywheel full power";
            fw.setRawPower(1);
            double i = fw.motorD().getCurrent(CurrentUnit.AMPS) + fw.motorS().getCurrent(CurrentUnit.AMPS);
            if (tm.seconds() < 0.3) { sumA += i; sumB += volts; n++; }
            minV = Math.min(minV, volts);
            maxI = Math.max(maxI, i);
            if (tm.seconds() > 1.0) { a[1] = sumA / n; a[2] = sumB / n; sumA = sumB = 0; n = 0; fw.stop(); next(); }
        } else if (phase == 3) {                              // coast down
            what = "flywheel coasting";
            fw.stop();
            if (tm.seconds() > 2.5) next();
        } else if (phase == 4) {                              // intake + stagiu current
            what = "intake + stagiu";
            intake.setPower(1);
            stagiu.setPower(1);
            if (tm.seconds() > 0.5) { sumC += intake.getCurrent(CurrentUnit.AMPS) + stagiu.getCurrent(CurrentUnit.AMPS); n++; }
            if (tm.seconds() > 1.5) { a[3] = sumC / Math.max(1, n); next(); }
        } else {
            double rest = a[0], iFly = a[1], vFly = a[2], drop = rest - minV;
            double r = iFly > 1 ? Math.max(0, rest - vFly) / iFly : Calib.BAT_R;
            Calib.BAT_R = r;
            Calib.I_FLY_START = iFly * 12 / Math.max(5, vFly);              // at 12 V
            Calib.save();
            String verdict = drop < 1 ? "GOOD - use in matches" : drop < 2 ? "WEAK - practice only" : "BAD - retire it";
            finish(3, String.format("BATTERY: rest %.2f V, flywheel start %.1f A, lowest %.2f V (drop %.2f V)%n"
                            + "R = %.0f mOhm (saved). VERDICT: %s%nIntake + stagiu: %.1f A%s%s", rest, iFly, minV, drop,
                    r * 1000, verdict, a[3], a[3] > 5 ? "  HIGH: jam / friction" : "  ok",
                    rest < 12.3 ? "\nRest under 12.3 V: battery not charged or old." : ""));
            return;
        }
        telemetry.addLine("BATTERY - balls OUT, robot still. Phase: " + what);
    }

    // ===================== 5: drive current =====================

    private void stepDrive() {
        if (phase == 0) {
            allOff();
            telemetry.addLine("DRIVE CURRENT - the robot drives ~40 cm (stick-up direction).");
            telemetry.addLine("Clear the space. CROSS = GO, SHARE = cancel.");
            if (cross) next();
            return;
        }
        if (phase == 1) {                                     // rest volts
            sumA += volts; n++;
            if (tm.seconds() > 0.3) { a[0] = sumA / n; sumA = 0; n = 0; a[4] = 0; a[6] = -1; maxI = 0; next(); }
        } else if (phase == 2) {                              // full power from stop
            stanga.setPower(DRIVE_DIR);
            dreapta.setPower(DRIVE_DIR);
            double i = Math.abs(stanga.getCurrent(CurrentUnit.AMPS)) + Math.abs(dreapta.getCurrent(CurrentUnit.AMPS));
            if (a[6] < 0) a[6] = tm.seconds();                        // power applied now (first loop)
            double t = tm.seconds() - a[6];
            if (t < 0.12) { sumA += i; sumB += volts; n++; }
            if (i > maxI) { maxI = i; t0 = t; a[5] = volts; }       // peak current, when, battery volts then
            if (t > 0.27) { sumC += i; sumD += t; a[4]++; }           // end of the test: current decayed
            minV = Math.min(minV, volts);
            if (t > 0.35) {
                stanga.setPower(0);
                dreapta.setPower(0);
                double iAvg = sumA / Math.max(1, n), vAvg = sumB / Math.max(1, n);
                double iEnd = sumC / Math.max(1, a[4]), tEnd = sumD / Math.max(1, a[4]);
                double r = iAvg > 1 ? Math.max(0, a[0] - vAvg) / iAvg : -1;
                double tau = iEnd > 0.5 && iEnd < maxI ? (tEnd - t0) / Math.log(maxI / iEnd) : Calib.DRIVE_TAU;
                tau = Math.max(0.03, Math.min(1.0, tau));
                // the first reading comes one loop late and at sagged volts: back to t = 0 and rest volts
                double iFull = maxI * Math.exp(t0 / tau) * 12 / Math.max(5, a[5]);   // at 12 V
                boolean ok = maxI > 1;
                if (ok) { Calib.I_DRIVE_FULL = iFull; Calib.DRIVE_TAU = tau; Calib.save(); }
                finish(4, ok
                        ? String.format("DRIVE: start current %.0f A (measured peak %.0f), wheels speed up in %.2f s (saved).%n"
                                + "Lowest %.2f V. Battery R from this test %.0f mOhm (battery step: %.0f).", iFull, maxI, tau, minV, r * 1000, Calib.BAT_R * 1000)
                        : "DRIVE FAILED: no current measured. Not saved.");
                if (!ok) done[4] = false;
                return;
            }
        }
        telemetry.addLine("DRIVE CURRENT - measuring...");
    }

    // ===================== 6: turret positions =====================

    private void stepTurret() {
        stickTurret();
        String saved = null;
        if (up)    saved = setPos("FRONT", 0);
        if (right) saved = setPos("RIGHT (init)", 1);
        if (down)  saved = setPos("BACK", 2);
        if (left)  saved = setPos("LEFT", 3);
        if (saved != null) msg = saved;
        telemetry.addLine("TURRET POSITIONS - left stick moves the turret.");
        telemetry.addLine("D-pad saves: UP = FRONT, RIGHT = RIGHT side (init), DOWN = BACK, LEFT = LEFT.");
        telemetry.addLine("FRONT / RIGHT / BACK must be EXACT (straight, 90 deg right, straight back):");
        telemetry.addLine("they also set the angle math for aiming. LEFT is only a target.");
        if (!msg.isEmpty()) telemetry.addLine(msg);
        telemetry.addData("Pot now", "%.3f V  =  %.0f deg", potV, Calib.turretDeg(potV));
        telemetry.addData("Saved F / R / B / L", "%.3f / %.3f / %.3f / %.3f", Calib.V_FRONT, Calib.V_RIGHT, Calib.V_BACK, Calib.V_LEFT);
    }

    private String setPos(String name, int which) {
        double f = Calib.V_FRONT, r = Calib.V_RIGHT, k = Calib.V_BACK;
        if (which == 0) f = potV; else if (which == 1) r = potV; else if (which == 2) k = potV;
        if (which < 3 && !(f < r && r < k)) return "NOT saved: must be FRONT < RIGHT < BACK in volts. Check the position.";
        if (which == 3 && (potV < Calib.V_LIMIT_MIN || potV > Calib.V_LIMIT_MAX)) return "NOT saved: outside the turret limits.";
        if (which == 0) Calib.V_FRONT = potV; else if (which == 1) Calib.V_RIGHT = potV;
        else if (which == 2) Calib.V_BACK = potV; else Calib.V_LEFT = potV;
        Calib.save();
        done[5] = true;
        return String.format("Saved %s = %.3f V", name, potV);
    }

    /** Left stick X moves the turret slowly; + raises pot V (toward RIGHT / BACK). Limits enforced. */
    private void stickTurret() {
        double p = gamepad1.left_stick_x * TURRET_STICK_POWER;
        if ((potV >= Calib.V_LIMIT_MAX && p > 0) || (potV <= Calib.V_LIMIT_MIN && p < 0)) p = 0;
        turret.setPower(p);
    }

    // ===================== 7: camera =====================

    private void stepCamera() {
        stickTurret();
        List<CellVision.Tag> now = new ArrayList<>();
        LLResult r = ll.getLatestResult();
        boolean fresh = false;
        if (r != null) {
            double st = r.getStaleness();
            fresh = st < lastStale;
            lastStale = st;
            if (r.isValid()) now = CellVision.tags(r, camBase >= 0 ? new int[]{camBase} : new int[]{30, 34, 38, 42});
        }
        double ref = phase < 2 ? REF_NEAR_CM : REF_FAR_CM;
        if (phase == 0 || phase == 2) {                       // waiting at the tape mark
            if (cross) {
                if (now.isEmpty()) msg = "No tags in view. Turn the turret with the left stick.";
                else { phase++; tm.reset(); frames = 0; for (double[] t : tagSum) java.util.Arrays.fill(t, 0); msg = ""; }
            }
        } else {                                              // collecting 1 s of frames
            if (fresh && !now.isEmpty()) {
                if (camBase < 0) camBase = now.get(0).base;
                for (CellVision.Tag t : now) {
                    if (t.base != camBase) continue;
                    tagSum[t.i][0] += t.range; tagSum[t.i][1] += t.tx; tagSum[t.i][2] += t.ty; tagSum[t.i][3]++;
                }
                frames++;
            }
            if (tm.seconds() > 1.0) {
                List<CellVision.Tag> avg = new ArrayList<>();
                for (int i = 0; i < 4; i++) {
                    double c = tagSum[i][3];
                    if (c >= Math.max(1, frames / 3.0)) avg.add(new CellVision.Tag(camBase, i, tagSum[i][0] / c, tagSum[i][1] / c, tagSum[i][2] / c));
                }
                if (avg.isEmpty()) { phase--; msg = "Lost the tags. Try again."; }
                else if (phase == 1) { near = avg; phase = 2; msg = "Near point done."; }
                else { far = avg; solveCamera(); return; }
            }
        }
        telemetry.addLine("CAMERA - move the robot so the TURRET CENTER is exactly");
        telemetry.addData("   distance (tape, floor) to the point under the tag cluster", "%.0f cm", ref);
        telemetry.addLine("Turn the turret to the tags (left stick). CROSS = measure.");
        if (!msg.isEmpty()) telemetry.addLine(msg);
        telemetry.addData("Tags / screen dist now", "%d / %.0f cm", now.size(),
                CellVision.distCm(now, Calib.CAM_PITCH_DEG, Calib.DIST_SCALE));
        if (phase == 1 || phase == 3) telemetry.addLine("MEASURING - hold still...");
    }

    /** Find camera pitch + range scale that make both tape distances right. */
    private void solveCamera() {
        double bestP = 0, bestS = 1, bestE = 1e9;
        for (double p = 0; p <= 80; p += 0.25) {
            double d1 = CellVision.distCm(near, p, 1), d2 = CellVision.distCm(far, p, 1);
            if (d1 <= 0 || d2 <= 0) continue;
            double s = (REF_NEAR_CM * d1 + REF_FAR_CM * d2) / (d1 * d1 + d2 * d2);   // least squares scale
            double e1 = CellVision.distCm(near, p, s) - REF_NEAR_CM, e2 = CellVision.distCm(far, p, s) - REF_FAR_CM;
            double e = Math.sqrt((e1 * e1 + e2 * e2) / 2);
            if (e < bestE) { bestE = e; bestP = p; bestS = s; }
        }
        double oldNear = CellVision.distCm(near, Calib.CAM_PITCH_DEG, Calib.DIST_SCALE);
        double oldFar = CellVision.distCm(far, Calib.CAM_PITCH_DEG, Calib.DIST_SCALE);
        boolean ok = bestE < 3 && bestP > 0.5 && bestP < 79.5 && bestS > 0.5 && bestS < 2;
        if (ok) { Calib.CAM_PITCH_DEG = bestP; Calib.DIST_SCALE = bestS; Calib.save(); }
        finish(6, String.format("CAMERA %s: pitch %.1f deg, scale %.3f, fit error %.1f cm%n"
                        + "Before: %.0f / %.0f cm. After: %.0f / %.0f cm (want %.0f / %.0f).%s",
                ok ? "SAVED" : "NOT saved", bestP, bestS, bestE, oldNear, oldFar,
                CellVision.distCm(near, bestP, bestS), CellVision.distCm(far, bestP, bestS), REF_NEAR_CM, REF_FAR_CM,
                ok ? "" : "\nFit is bad: check the tape distances and that the same tags were seen."));
        if (!ok) done[6] = false;
    }

    // ===================== helpers =====================

    private void next() { phase++; tm.reset(); }

    private void allOff() {
        fw.setBrake(false);
        fw.stop();
        for (DcMotor m : new DcMotor[]{stanga, dreapta, intake, stagiu, turret}) m.setPower(0);
    }

    private void readButtons() {
        boolean[] now = {gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_left, gamepad1.dpad_right,
                gamepad1.cross, gamepad1.circle, gamepad1.share, gamepad1.left_bumper, gamepad1.right_bumper};
        boolean[] e = new boolean[now.length];
        for (int i = 0; i < now.length; i++) { e[i] = now[i] && !prev[i]; prev[i] = now[i]; }
        up = e[0]; down = e[1]; left = e[2]; right = e[3]; cross = e[4]; circle = e[5]; share = e[6]; l1 = e[7]; r1 = e[8];
    }

    @Override
    public void stop() {
        if (fw != null) allOff();
        if (ll != null) ll.stop();
    }
}
