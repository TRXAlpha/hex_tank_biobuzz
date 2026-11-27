package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Calib;
import org.firstinspires.ftc.teamcode.CellVision;
import org.firstinspires.ftc.teamcode.Flywheel;
import org.firstinspires.ftc.teamcode.Power;

import java.util.List;

/*
 * TELEOP AUTO-AIM RED (copy of the BLUE file: a fix goes in BOTH). One fixed flywheel speed (Calib.SHOT_SPEED). The turret aims by itself or holds a fixed spot.
 *
 * DRIVER (PS5):
 *   Left stick Y + right stick X   drive (battery limiter: Power)
 *   L1 hold        intake
 *   R2 hold        SHOOT: feeds when the flywheel is at speed and the turret is steady
 *                  (AIM: on target, or no target yet / FIXED: at its spot)
 *   Triangle       auto-fire ON/OFF (starts OFF). Works only in AIM mode: 2+ tags + steady aim + speed
 *   Touchpad       AIM <-> FIXED. AIM -> FIXED goes to FRONT
 *   D-pad          FIXED spot: UP = FRONT, DOWN = BACK, LEFT = LEFT, RIGHT = RIGHT (auto init spot)
 *   Circle hold    unjam (intake + stagiu reverse)
 *   Square         flywheel on/off (on at start)
 *   Share          forget the CELL and scan again
 * Alliance: RED (fixed in this file).
 *
 * AIM mode, every new camera frame: our CELL center (CellVision) -> FIELD point with the Pinpoint pose from
 * when the picture was taken. Between frames / without tags the turret aims at that point with odometry.
 * Before the first sighting the turret sweeps its range. All physical values come from Calib (ROBOT CALIBRATION).
 */
@Config
@TeleOp(name = "TELEOP AUTO-AIM RED", group = "turela")
public class TeleopAutoAimRed extends OpMode {

    // ---------- vision ----------
    public static int PIPELINE = 2;
    public static double SCAN_DEG_S = 60;        // turret sweep speed while looking for the first tags
    public static double LOST_MS = 300;          // tags count as "seen" for this long after the last frame
    public static double CENTER_ALPHA = 0.5;     // CELL point smoothing (1 = none)
    public static double JUMP_MM = 400;          // new point this far away = replace, do not smooth

    // ---------- turret control ----------
    public static boolean MOTOR_REVERSED = false;
    public static double MAX_POWER = 1, kP = 0.006, kD = 0.0002, kS = 0.2, kV = 0.0015;
    public static double TOLERANCE_DEG = 1.5, FILTER_ALPHA = 0.4;
    public static double CAM_ONLY_MAX_POWER = 0.35;  // turret power cap when aiming on the camera with a BAD pot (no limits known)
    public static double CAM_PULSE_POWER = 0.35;     // bad pot: push, stop, look at a NEW camera frame, push again
    public static double CAM_MS_PER_DEG_START = 5;
    public static double POT_GOOD_MS = 500;          // pot counts as good again only after this long without a bad reading   // first guess of push time per degree (learned while it runs)
    public static double GLITCH_V = 0.4;          // pot jump bigger than this in 1 loop = bad reading (try 0.3-0.4)
    public static int GLITCH_LOOPS = 3;           // ignore up to this many loops, then accept the new value

    // ---------- shooting ----------
    public static double AIM_TOL_DEG = 3;
    public static double AIM_SETTLE_MS = 100;    // must stay on target this long before a shot
    public static int FIRE_MIN_TAGS = 2;
    public static boolean AUTO_FIRE_AT_START = false;   // Triangle toggles it during the match

    private DcMotor stanga, dreapta, turret, intake, stagiu;
    private AnalogInput pot;
    private Limelight3A ll;
    private GoBildaPinpointDriver odo;
    private Flywheel fw;
    private Power power;
    private final boolean red = true;       // alliance of this file
    private boolean autoFire, flyWasOn;
    private double scanDir = 1, onSince = -1;

    // turret mode
    private boolean fixed = false;
    private double fixedV;
    private String fixedName = "";

    // aim
    private boolean haveAim = false;
    private double fx, fy;                // mm, CELL center on the field (odometry frame)
    private double dist = 0;              // cm, turret pivot -> CELL center
    private double camDist = 0;           // cm, same, from the last camera frame only
    private double camBear = 0;           // deg, CELL center seen from the turret (+ = left), last frame
    private double camHead, camPan;       // robot heading (rad) and turret angle (deg) when that frame was taken
    private boolean camPanOk;             // pot was good when that frame was taken
    private String aimSrc = "";
    private double camFrameT = -1e9, lastPotBad = -1e9;      // when the last camera frame was taken (clock ms)
    private double pulseUntil = 0, pulseSign = 0, pulseMs = 0, errBeforePulse = 0, msPerDeg = -1;
    private int tags = 0;
    private double lastSeen = -1e9, lastStale = 1e9;

    // turret
    private double v, deg, rawDeg, lastDeg, target, minDeg, maxDeg;
    private boolean potOk = true, saturated = false;
    private double goodRaw = -1;                  // last accepted pot voltage
    private int badLoops = 0, glitches = 0, resyncs = 0;

    // buttons (edge detect): touchpad, up, down, left, right, square, triangle
    private final boolean[] prev = new boolean[7];

    // short history (time, pose, turret) so a camera frame uses the pose from when it was taken
    private static final int H = 64;
    private final double[] hT = new double[H], hX = new double[H], hY = new double[H],
            hHead = new double[H], hPan = new double[H];
    private final boolean[] hOk = new boolean[H];
    private int hI = 0;

    private final ElapsedTime clock = new ElapsedTime();
    private double lastMs = 0;
    private double loopMs = 0, loopAvg = 0, loopMax = 0, loopMaxShown = 0, loopWinStart = 0;   // ms

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        Calib.load();
        stanga = hardwareMap.dcMotor.get("stanga");
        dreapta = hardwareMap.dcMotor.get("dreapta");
        dreapta.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.dcMotor.get("intake");
        stagiu = hardwareMap.dcMotor.get("stagiu");
        stagiu.setDirection(DcMotorSimple.Direction.REVERSE);
        turret = hardwareMap.dcMotor.get("turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(MOTOR_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        for (DcMotor m : new DcMotor[]{stanga, dreapta, intake, stagiu, turret}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        pot = hardwareMap.get(AnalogInput.class, "potTurela");

        ll = hardwareMap.get(Limelight3A.class, "limelight");
        ll.setPollRateHz(100);
        ll.pipelineSwitch(PIPELINE);
        ll.start();

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(-44.3, -6.6, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();                // pose = 0,0,0 at INIT (keep the robot still)

        fw = new Flywheel(hardwareMap);      // nothing moves in INIT
        power = new Power(hardwareMap);

        double a = Calib.turretDeg(Calib.V_LIMIT_MIN), b = Calib.turretDeg(Calib.V_LIMIT_MAX);
        minDeg = Math.min(a, b);
        maxDeg = Math.max(a, b);
        v = pot.getVoltage();
        deg = rawDeg = lastDeg = target = Calib.turretDeg(v);
    }

    @Override
    public void init_loop() {
        double volts = power.read();
        telemetry.addData("ALLIANCE", red ? "RED" : "BLUE");
        telemetry.addData("Calibration", Calib.fromFile ? "loaded" : "DEFAULTS - run ROBOT CALIBRATION");
        telemetry.addData("Shot speed", "%.0f", Calib.SHOT_SPEED);
        telemetry.addData("Battery", "%.2f V%s", volts, volts < 12.3 ? "  <- CHANGE IT" : "");
        telemetry.update();
    }

    @Override
    public void start() {
        fw.setOn(true);
        autoFire = AUTO_FIRE_AT_START;
        readButtons();                       // buttons held from INIT do not count as presses
        clock.reset();
        lastMs = 0;
    }

    @Override
    public void loop() {
        double now = clock.milliseconds(), dt = (now - lastMs) / 1000.0;
        lastMs = now;
        if (dt <= 0) return;
        loopMs = dt * 1000;
        loopAvg = loopAvg == 0 ? loopMs : loopAvg + 0.05 * (loopMs - loopAvg);   // smooth average
        loopMax = Math.max(loopMax, loopMs);
        if (now - loopWinStart >= 1000) { loopMaxShown = loopMax; loopMax = 0; loopWinStart = now; }   // worst loop, last 1 s
        double volts = power.read();
        boolean[] pr = readButtons();

        // ---------- drive (through the battery limiter) ----------
        double fwd = gamepad1.left_stick_y, turn = -gamepad1.right_stick_x;
        double m = Math.max(1, Math.abs(fwd) + Math.abs(turn));
        double[] d = power.drive((fwd + turn) / m, (fwd - turn) / m, dt);
        stanga.setPower(d[0]);
        dreapta.setPower(d[1]);

        // ---------- sensors ----------
        odo.update();
        Pose2D pose = odo.getPosition();
        double x = pose.getX(DistanceUnit.MM), y = pose.getY(DistanceUnit.MM);
        double head = pose.getHeading(AngleUnit.RADIANS);
        double omega = odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        readTurret();
        hT[hI] = now; hX[hI] = x; hY[hI] = y; hHead[hI] = head; hPan[hI] = rawDeg; hOk[hI] = potOk;
        hI = (hI + 1) % H;

        // ---------- vision ----------
        LLResult r = ll.getLatestResult();
        if (r != null) {
            double stale = r.getStaleness();
            boolean fresh = stale < lastStale;      // staleness drops when a new frame arrives
            lastStale = stale;
            if (fresh && r.isValid()) {
                see(r, now - stale - r.getCaptureLatency() - r.getTargetingLatency());
            }
        }
        if (now - lastSeen > LOST_MS) tags = 0;
        if (haveAim) dist = Math.hypot(fx - x, fy - y) / 10.0;

        // ---------- turret mode: touchpad AIM <-> FIXED, D-pad = fixed spot ----------
        if (pr[0]) { if (fixed) fixed = false; else setFixed("FRONT", Calib.V_FRONT); }
        if (pr[1]) setFixed("FRONT", Calib.V_FRONT);
        if (pr[2]) setFixed("BACK", Calib.V_BACK);
        if (pr[3]) setFixed("LEFT", Calib.V_LEFT);
        if (pr[4]) setFixed("RIGHT", Calib.V_RIGHT);
        if (gamepad1.share) { haveAim = false; tags = 0; }    // forget -> scan again (AIM mode)

        // ---------- turret target ----------
        if (!potOk) lastPotBad = now;
        boolean potGood = potOk && now - lastPotBad > POT_GOOD_MS;    // spiking pot: stay on the camera
        // AIM + tags visible: the error comes from the CAMERA (it sits on the turret), not from the pot.
        // The pot is only used for the end stops, and only while it reads well.
        double tv = 0, camErr = Double.NaN;
        boolean camAim = !fixed && now - lastSeen <= LOST_MS;          // works even if the pot was never good
        if (camAim) {
            camErr = camBear - Math.toDegrees(head - camHead);             // robot turned since the frame
            if (potOk && camPanOk) camErr -= deg - camPan;                 // turret moved since the frame
            tv = -Math.toDegrees(omega);
        }
        if (fixed) {
            target = Math.max(minDeg, Math.min(maxDeg, Calib.turretDeg(fixedV)));
            saturated = false;
        } else if (camAim) {
            double want = potOk ? deg + camErr : camErr;
            target = potOk ? reachable(want) : want;
            saturated = potOk && Math.abs(AngleUnit.normalizeDegrees(want - target)) > 0.5;
            if (saturated) tv = 0;
        } else if (haveAim) {
            double want = AngleUnit.normalizeDegrees(Math.toDegrees(Math.atan2(fy - y, fx - x) - head));
            target = reachable(want);
            saturated = Math.abs(AngleUnit.normalizeDegrees(want - target)) > 0.5;
            if (!saturated) tv = -Math.toDegrees(omega);   // turn against the robot's rotation
        } else {                                           // never seen the CELL: sweep to find it
            if (target >= maxDeg - 5) scanDir = -1;
            else if (target <= minDeg + 5) scanDir = 1;
            target = Math.max(minDeg, Math.min(maxDeg, target + scanDir * SCAN_DEG_S * dt));
            saturated = false;
            tv = scanDir * SCAN_DEG_S;
        }
        double err = camAim ? (potOk ? target - deg : camErr) : target - deg;
        aimSrc = fixed ? (potOk ? "POT (fixed spot)" : "HOLD - pot bad, fixed spots need it")
               : camAim ? (potGood ? "CAMERA" : "CAMERA ONLY - pot bad, push + look")
               : haveAim ? (potOk ? "POT + odometry (no tags)" : "HOLD - pot bad, no tags")
               : (potOk ? "SCAN" : "HOLD - pot bad, cannot scan");
        if (camAim && !potGood) camOnlyTurret(now, camErr, tv);
        else driveTurret(dt, tv, err, false);

        // ---------- flywheel: one fixed speed ----------
        if (pr[5]) fw.setOn(!fw.isOn());
        if (pr[6]) autoFire = !autoFire;
        boolean hold = gamepad1.right_trigger > 0.5;
        if (hold) fw.setOn(true);
        if (fw.isOn() && !flyWasOn && fw.getVelocity() < 100) power.learnFromFlywheelStart();   // battery check
        flyWasOn = fw.isOn();
        fw.setTarget(Calib.SHOT_SPEED);
        fw.update(volts);

        // ---------- shoot decision ----------
        double aimErr = camAim && !potGood ? camErr : err;
        boolean onTarget = (fixed || haveAim || camAim) && !saturated && Math.abs(aimErr) <= AIM_TOL_DEG && (potOk || camAim);
        if (!onTarget) onSince = -1; else if (onSince < 0) onSince = now;
        boolean steady = onTarget && now - onSince >= AIM_SETTLE_MS;
        boolean ok = false;
        String why;
        if (!fw.isOn())                    why = "flywheel OFF (Square)";
        else if (fixed)                    why = "FIXED " + fixedName + ": auto-fire off, hold R2";
        else if (!haveAim && !camAim)      why = "SCANNING for tags";
        else if (tags < FIRE_MIN_TAGS)     why = "looking for tags";
        else if (saturated)                why = "turret at limit - turn robot";
        else if (!steady)                  why = "aiming";
        else if (!fw.isAtSpeed())          why = "speeding up";
        else { why = "FIRE"; ok = true; }
        boolean r2Ready = fixed ? steady : ((!haveAim && !camAim) || steady);
        boolean feed = fw.isAtSpeed() && ((autoFire && !fixed && ok) || (hold && r2Ready));

        // ---------- intake + stagiu (one write each per loop) ----------
        if (gamepad1.circle)  { intake.setPower(-1); stagiu.setPower(-1); }
        else if (feed)        { intake.setPower(1);  stagiu.setPower(1); }
        else                  { intake.setPower(gamepad1.left_bumper ? 1 : 0); stagiu.setPower(0); }

        // ---------- screen ----------
        telemetry.addData("TURRET (touchpad)", fixed ? "FIXED " + fixedName : (haveAim ? "AIM" : "AIM - scanning"));
        telemetry.addData("AUTO-FIRE (Triangle)", autoFire ? (fixed ? "ON (only in AIM)" : "ON") : "OFF - hold R2");
        telemetry.addData("SHOOT", hold ? (feed ? "R2: FIRING" : "R2: waiting for " + (fw.isAtSpeed() ? "turret" : "speed")) : why);
        telemetry.addData("Speed", "%.0f / %.0f", fw.getVelocity(), fw.getTarget());
        telemetry.addData("Tags / dist (cam)", "%d / %.0f cm (%.0f)", tags, dist, camDist);
        telemetry.addData("Turret aim", aimSrc);
        telemetry.addData("Turret", "%.1f -> %.1f deg, error %.1f%s", deg, target, err, potOk ? "" : "  POT BAD");
        telemetry.addData("Pot glitches / resyncs", "%d / %d", glitches, resyncs);
        telemetry.addData("Battery / R / drive cut", "%.2f V / %.0f mOhm / %.0f %%%s", volts, power.ohms() * 1000,
                power.cut() * 100, volts < Power.V_FLOOR ? "  LOW" : "");
        telemetry.addData("Odometry x / y / heading", "%.0f / %.0f mm / %.0f deg", x, y, Math.toDegrees(head));
        telemetry.addData("Loop now / avg / max(1s)", "%.0f / %.1f / %.0f ms  (%.0f Hz)",
                loopMs, loopAvg, loopMaxShown, 1000.0 / Math.max(loopAvg, 1e-3));
        telemetry.update();
    }

    private void setFixed(String name, double volts) {
        fixed = true;
        fixedName = name;
        fixedV = volts;
    }

    /** Edge detect: {touchpad, up, down, left, right, square, triangle} pressed this loop. */
    private boolean[] readButtons() {
        boolean[] now = {gamepad1.touchpad, gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_left,
                gamepad1.dpad_right, gamepad1.square, gamepad1.triangle};
        boolean[] e = new boolean[now.length];
        for (int i = 0; i < now.length; i++) { e[i] = now[i] && !prev[i]; prev[i] = now[i]; }
        return e;
    }

    // ===================== vision =====================

    /** One camera frame -> CELL center -> field point (fx, fy). tFrame = when the picture was taken. */
    private void see(LLResult r, double tFrame) {
        List<CellVision.Tag> t = CellVision.tags(r, red ? new int[]{30, 34} : new int[]{38, 42});  // our 2 CELLS
        double[] c = CellVision.center(t, Calib.CAM_PITCH_DEG, Calib.DIST_SCALE);
        if (c == null) return;
        int j = (hI - 1 + H) % H;                                   // history entry at frame time
        for (int s = 0; s < H - 1 && hT[j] > tFrame; s++) j = (j - 1 + H) % H;

        camDist = Math.hypot(c[0], c[1]) / 10.0;
        camBear = Math.toDegrees(Math.atan2(c[1], c[0]));           // + = left of the turret
        camHead = hHead[j];
        camPan = hPan[j];
        camPanOk = hOk[j];
        camFrameT = tFrame;
        haveAim = haveAim || camPanOk;
        tags = t.size();
        lastSeen = clock.milliseconds();
        if (!camPanOk) return;                                      // bad pot then: do not move the field point
        double a = hHead[j] + Math.toRadians(hPan[j]);              // turret direction on the field
        double nx = hX[j] + c[0] * Math.cos(a) - c[1] * Math.sin(a);
        double ny = hY[j] + c[0] * Math.sin(a) + c[1] * Math.cos(a);
        if (!haveAim || Math.hypot(nx - fx, ny - fy) > JUMP_MM) { fx = nx; fy = ny; }
        else { fx += CENTER_ALPHA * (nx - fx); fy += CENTER_ALPHA * (ny - fy); }
        haveAim = true;
    }

    // ===================== turret =====================

    private void readTurret() {
        double raw = pot.getVoltage();
        boolean inRange = raw > 0.03 && raw < 3.25;
        boolean jump = goodRaw >= 0 && Math.abs(raw - goodRaw) > GLITCH_V;
        if (!inRange || jump) {
            badLoops++;
            glitches++;
            if (!inRange || badLoops < GLITCH_LOOPS) { potOk = false; return; }   // ignore -> driveTurret sets power 0
            v = raw;                         // same new value GLITCH_LOOPS loops in a row -> it is real
            resyncs++;
        }
        badLoops = 0;
        potOk = true;
        goodRaw = raw;
        v += FILTER_ALPHA * (raw - v);
        rawDeg = Calib.turretDeg(raw);     // unfiltered -> vision history
        deg = Calib.turretDeg(v);          // filtered -> control
    }

    /**
     * Bad pot, tags visible: the camera is the only angle sensor and it is ~50 ms late, so a normal
     * controller overshoots and wobbles. Instead: short push sized to the error, stop, wait for a frame
     * taken AFTER the push, push again. The push time per degree is learned from what each push did.
     */
    private void camOnlyTurret(double now, double err, double tv) {
        if (msPerDeg < 0) msPerDeg = CAM_MS_PER_DEG_START;
        double p = kV * tv;                                          // keep turning against the robot
        if (now < pulseUntil) {
            p += pulseSign * CAM_PULSE_POWER;
        } else if (camFrameT > pulseUntil + 20) {                    // new picture since the push ended
            if (pulseMs > 0) {                                       // learn from the last push
                double moved = (errBeforePulse - err) * pulseSign;
                if (moved > 0.3) msPerDeg += 0.5 * (pulseMs / moved - msPerDeg);
                msPerDeg = Math.max(1, Math.min(50, msPerDeg));
                pulseMs = 0;
            }
            if (Math.abs(err) > AIM_TOL_DEG / 2) {
                pulseMs = Math.max(15, Math.min(150, Math.abs(err) * msPerDeg));
                pulseSign = Math.signum(err);
                errBeforePulse = err;
                pulseUntil = now + pulseMs;
                p += pulseSign * CAM_PULSE_POWER;
            }
        }
        p = Math.max(-CAM_ONLY_MAX_POWER, Math.min(CAM_ONLY_MAX_POWER, p));
        if (potOk && ((v >= Calib.V_LIMIT_MAX && -p > 0) || (v <= Calib.V_LIMIT_MIN && -p < 0))) p = 0;   // end stop
        turret.setPower(-p);                                         // + deg (left) = lower pot V
    }

    /** err = turret error (deg, + = turn left). camOnly = aiming on the camera with a bad pot. */
    private void driveTurret(double dt, double tv, double err, boolean camOnly) {
        double vel = potOk ? (deg - lastDeg) / dt : tv;             // no pot: no damping term
        if (potOk) lastDeg = deg;
        double p = kV * tv;
        if (Math.abs(err) > TOLERANCE_DEG) p += kP * err + kD * (tv - vel) + Math.copySign(kS, err);
        double max = camOnly ? CAM_ONLY_MAX_POWER : MAX_POWER;
        p = -Math.max(-max, Math.min(max, p));                      // + deg (left) = lower pot V
        if (potOk && ((v >= Calib.V_LIMIT_MAX && p > 0) || (v <= Calib.V_LIMIT_MIN && p < 0))) p = 0;
        if (!potOk && !camOnly) p = 0;                              // bad pot and no camera aim: hold
        turret.setPower(p);
    }

    /** Closest reachable turret angle to want (tries want and want +/- 360). */
    private double reachable(double want) {
        double best = 0, bestErr = Double.MAX_VALUE;
        for (int k = -1; k <= 1; k++) {
            double a = want + 360.0 * k;
            double e = a < minDeg ? minDeg - a : (a > maxDeg ? a - maxDeg : 0);
            if (e < bestErr) { bestErr = e; best = Math.max(minDeg, Math.min(maxDeg, a)); }
        }
        return best;
    }

    @Override
    public void stop() {
        for (DcMotor mo : new DcMotor[]{stanga, dreapta, intake, stagiu, turret}) if (mo != null) mo.setPower(0);
        if (fw != null) fw.stop();
        if (ll != null) ll.stop();
    }
}