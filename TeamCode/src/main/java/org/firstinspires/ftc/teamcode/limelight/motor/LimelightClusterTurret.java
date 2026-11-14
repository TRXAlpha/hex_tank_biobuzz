package org.firstinspires.ftc.teamcode.limelight.motor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/**
 * PAN-ONLY cluster tracker (Limelight 3A) on the MOTOR turret (motor + pot, no encoder).
 * Cluster estimation = identical to LimelightClusterPan (servo version).
 * Changes vs servo version:
 *   - pan servo -> motor + pot PID with feedforward (turelaFieldLock controller)
 *   - pan history stores the MEASURED turret angle (pot), not the commanded one -> no servo-lag guess
 *   - pan limits come from the pot voltage limits; target picked inside the reachable arc
 *   - tilt servo removed (fixed camera pitch, CAM_PITCH_DEG)
 *
 * Pan angle convention (same as servo version): robot frame, CCW +, 0 = robot front.
 */
@Config
@TeleOp(name = "Limelight Cluster Turret", group = "turela")
public class LimelightClusterTurret extends OpMode {

    // ================= CAMERA / MOUNT =================
    public static double MOUNT_X_MM = 0, MOUNT_Y_MM = 0;   // turret pivot from robot center
    public static double CAM_FWD_MM = 30;                   // camera ahead of pivot
    public static double CAM_PITCH_DEG = 0;

    // ================= CLUSTER =================
    public static int FIRST_TAG = 38;
    public static double TAG_SPACING_MM = 150;
    public static double DIR_ALPHA = 0.2;
    public static double SIGN_CONF_SIN = 0.5;
    public static double SLOPE_DECAY = 0.9;

    // ================= TRACKING =================
    public static int PIPELINE = 2;
    public static double CENTER_ALPHA = 0.3;
    public static double JUMP_RESET_MM = 400;
    public static double EXTRA_LATENCY_MS = 10;
    public static double PAN_LAG_MS = 0;          // history is measured angle -> ~0
    public static double LOOKAHEAD_MS = 30;
    public static double MIN_DIST_MM = 200, MAX_DIST_MM = 6000;

    // ================= TURRET: POT CALIBRATION =================
    public static double V_FRONT  = 0.663;
    public static double V_CENTER = 1.030;
    public static double V_BACK   = 1.500;
    public static double V_LIMIT_MIN = 0.5;
    public static double V_LIMIT_MAX = 1.75;
    public static boolean LATERAL_IS_LEFT = false;   // must be right for BOTH aiming and vision math

    // ================= TURRET: CONTROL =================
    public static boolean MOTOR_REVERSED = false;   // + power raises pot V
    public static double MAX_POWER = 1;
    public static double kP = 0.006;
    public static double kD = 0.0002;
    public static double kS = 0.2;
    public static double kV = 0.0015;               // power per deg/s of target motion
    public static double TOLERANCE_DEG = 1.5;
    public static double FILTER_ALPHA = 0.4;
    public static double FF_ALPHA = 0.3;            // smoothing of target velocity
    public static double FF_JUMP_DEG = 15;          // target jump bigger than this = no FF spike

    // ================= HARDWARE =================
    private DcMotor stanga, dreapta, turret;
    private AnalogInput pot;
    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;
    private FtcDashboard dashboard;

    // ================= CLUSTER STATE =================
    private static final int N = 4;
    private boolean haveCenter = false, haveU = false, uAssumed = false;
    private double cx, cy;
    private double ux = 0, uy = 1;
    private double slopeAcc = 0;

    private double lastStaleness = Double.MAX_VALUE;
    private double lastSeenMs = -1e9;
    private double lastRange, lastC;
    private int lastCnt = 0;
    private String lastTags = "-", lastTx = "-", lastYaw = "-", lastNote = "";

    // ================= TURRET STATE =================
    private double v, deg, rawDeg, lastDeg;
    private boolean potOk = true;
    private double minDeg, maxDeg;
    private double holdDeg, desired, lastDesired, target, targetVel;
    private boolean saturated = false;
    private double motorPower = 0;

    private final ElapsedTime runtime = new ElapsedTime();
    private double lastLoopMs = 0;

    private static final int HIST = 256;
    private final double[] hT = new double[HIST], hX = new double[HIST], hY = new double[HIST],
            hH = new double[HIST], hPan = new double[HIST];
    private int hIdx = 0, hCount = 0;

    // =====================================================================

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        turret = hardwareMap.get(DcMotor.class, "turret");
        pot    = hardwareMap.get(AnalogInput.class, "potTurela");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();
        dashboard.startCameraStream(limelight, 40);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(0, 0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        stanga = hardwareMap.get(DcMotor.class, "stanga");
        dreapta = hardwareMap.get(DcMotor.class, "dreapta");
        stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        v = pot.getVoltage();
    }

    @Override
    public void init_loop() {
        computeLimits();
        v = pot.getVoltage();
        telemetry.addData("Pot V", "%.3f", v);
        telemetry.addData("Turret deg (robot)", "%.1f", turretDeg(v));
        telemetry.addData("Reach", "%.0f .. %.0f", minDeg, maxDeg);
        telemetry.update();
    }

    @Override
    public void start() {
        turret.setDirection(MOTOR_REVERSED ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
        computeLimits();
        v = pot.getVoltage();
        deg = rawDeg = lastDeg = turretDeg(v);
        holdDeg = desired = lastDesired = target = deg;   // hold until first tag
        targetVel = 0;
        runtime.reset();
        lastLoopMs = 0;
    }

    @Override
    public void loop() {
        double now = runtime.milliseconds();
        double loopMs = now - lastLoopMs;
        lastLoopMs = now;
        double dt = loopMs / 1000.0;
        if (dt <= 0) return;

        drive();

        // ---------- ODOMETRY ----------
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        double rx = pose.getX(DistanceUnit.MM);
        double ry = pose.getY(DistanceUnit.MM);
        double rh = pose.getHeading(AngleUnit.RADIANS);
        double omega = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

        // ---------- TURRET ANGLE (measured) ----------
        readTurret();

        // ---------- VISION ----------
        LLResult r = limelight.getLatestResult();
        double staleness = -1;
        if (r != null) {
            staleness = r.getStaleness();
            boolean newFrame = staleness < lastStaleness;
            lastStaleness = staleness;

            if (newFrame && r.isValid()) {
                double agoMs = staleness + r.getCaptureLatency() + r.getTargetingLatency()
                        + EXTRA_LATENCY_MS;
                int iPose = lookup(now - agoMs);
                int iPan  = lookup(now - agoMs - PAN_LAG_MS);
                if (iPose >= 0 && iPan >= 0) processFrame(r, iPose, iPan, now);
            }
        }

        // ---------- AIM (robot-frame pan angle) ----------
        if (haveCenter) {
            double hAhead = rh + omega * LOOKAHEAD_MS / 1000.0;
            double mx = rx + MOUNT_X_MM * Math.cos(rh) - MOUNT_Y_MM * Math.sin(rh);
            double my = ry + MOUNT_X_MM * Math.sin(rh) + MOUNT_Y_MM * Math.cos(rh);
            desired = Math.toDegrees(wrap(Math.atan2(cy - my, cx - mx) - hAhead));
        } else {
            desired = holdDeg;
        }

        // target velocity for feedforward (rotation + translation), spike-guarded
        double dDes = AngleUnit.normalizeDegrees(desired - lastDesired);
        lastDesired = desired;
        double rawVel = Math.abs(dDes) > FF_JUMP_DEG ? 0 : dDes / dt;
        targetVel += FF_ALPHA * (rawVel - targetVel);

        target = pickReachable(desired);
        saturated = Math.abs(AngleUnit.normalizeDegrees(desired - target)) > 0.5;

        driveTurret(dt);
        record(now, rx, ry, rh, rawDeg);

        // ---------- TELEMETRY ----------
        double sinceSeen = (now - lastSeenMs) / 1000.0;
        telemetry.addData("Mode", !haveCenter ? "NO TAG YET"
                : sinceSeen < 0.2 ? "VISION LOCK" : String.format("ODOMETRY ONLY (%.1fs)", sinceSeen));
        telemetry.addData("Tags / count", "%s / %d  (%s)", lastTags, lastCnt, lastNote);
        telemetry.addData("Per-tag tx (deg)", lastTx);
        telemetry.addData("Per-tag yaw (deg, debug)", lastYaw);
        telemetry.addData("Row dir", !haveU ? "none" : String.format("%.1f deg  %s",
                Math.toDegrees(Math.atan2(uy, ux)), uAssumed ? "ASSUMED/low-conf" : "measured"));
        telemetry.addData("View c / angle", "%.2f / %.0f deg  (cfg spacing %.0f)",
                lastC, Math.toDegrees(Math.acos(clamp(lastC, -1, 1))), TAG_SPACING_MM);
        telemetry.addData("Center (mm) / range", "x %.0f  y %.0f  / %.0f", cx, cy, lastRange);
        telemetry.addData("Pan desired / target / actual", "%.1f / %.1f / %.1f", desired, target, deg);
        telemetry.addData("SATURATED", saturated ? "PAN (blind zone)" : "");
        telemetry.addData("Target vel / power", "%.0f deg/s / %.2f", targetVel, motorPower);
        telemetry.addData("Pot V / ok", "%.3f / %b", v, potOk);
        telemetry.addData("Robot", "x %.0f  y %.0f  h %.1f", rx, ry, Math.toDegrees(rh));
        telemetry.addData("Staleness", "%.0f ms", staleness);
        telemetry.addData("Loop", "%.1f ms", loopMs);
        telemetry.update();
    }

    // ===================== TURRET =====================

    private void readTurret() {
        double raw = pot.getVoltage();
        potOk = raw > 0.03 && raw < 3.25;
        if (!potOk) return;
        v += FILTER_ALPHA * (raw - v);
        rawDeg = turretDeg(raw);     // unfiltered -> vision history (no filter lag)
        deg = turretDeg(v);          // filtered -> control
    }

    private void driveTurret(double dt) {
        double error = target - deg;
        double turretVel = (deg - lastDeg) / dt;
        lastDeg = deg;
        double tv = saturated ? 0 : targetVel;

        double power = kV * tv;
        if (Math.abs(error) > TOLERANCE_DEG) {
            power += kP * error + kD * (tv - turretVel) + Math.copySign(kS, error);
        }
        power = clamp(power, -MAX_POWER, MAX_POWER);

        motorPower = LATERAL_IS_LEFT ? power : -power;   // deg space -> "+ raises pot V"
        if ((v >= V_LIMIT_MAX && motorPower > 0) || (v <= V_LIMIT_MIN && motorPower < 0)) motorPower = 0;
        if (!potOk) motorPower = 0;

        turret.setPower(motorPower);
    }

    /** Pot voltage -> turret deg, robot frame (0 front, +-90 lateral, +-180 back, CCW +). */
    private double turretDeg(double volts) {
        double p  = potAngleDeg(volts);
        double pC = potAngleDeg(V_CENTER);
        double old;   // +90 front, 0 center, -90 back
        if (p <= pC) old = (p - pC) / (pC - potAngleDeg(V_BACK)) * 90.0;
        else         old = (p - pC) / (potAngleDeg(V_FRONT) - pC) * 90.0;
        double d = 90.0 - old;
        return LATERAL_IS_LEFT ? d : -d;
    }

    private void computeLimits() {
        double a = turretDeg(V_LIMIT_MIN), b = turretDeg(V_LIMIT_MAX);
        minDeg = Math.min(a, b);
        maxDeg = Math.max(a, b);
    }

    /** 360-equivalent of 'want' inside [minDeg, maxDeg]; if none, the nearest limit. */
    private double pickReachable(double want) {
        want = AngleUnit.normalizeDegrees(want);
        double best = 0, bestDist = Double.MAX_VALUE;
        for (int k = -1; k <= 1; k++) {
            double c = want + 360.0 * k;
            double d = c < minDeg ? minDeg - c : (c > maxDeg ? c - maxDeg : 0);
            if (d < bestDist) { bestDist = d; best = clamp(c, minDeg, maxDeg); }
        }
        return best;
    }

    /** REV pot on FTC hub (nonlinear): 3.3V->0, 1.1V->135, 0V->270 pot deg. */
    private static double potAngleDeg(double volts) {
        if (volts < 1e-3) return 270.0;
        double b = 270.0 * volts + 445.5;
        double c = 120285.0 - 36450.0 * volts;
        return (b - Math.sqrt(b * b - 4.0 * volts * c)) / (2.0 * volts);
    }

    // ===================== VISION (unchanged from servo version) =====================

    private void processFrame(LLResult r, int iPose, int iPan, double now) {
        double h = hH[iPose];
        double panR = Math.toRadians(hPan[iPan]);

        boolean[] seen = new boolean[N];
        double[] bearing = new double[N], horiz = new double[N], txDeg = new double[N], yawDeg = new double[N];
        int cnt = 0;

        for (LLResultTypes.FiducialResult f : r.getFiducialResults()) {
            int i = f.getFiducialId() - FIRST_TAG;
            if (i < 0 || i >= N || seen[i]) continue;

            Position p = f.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.MM);
            double range = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            if (range <= MIN_DIST_MM || range >= MAX_DIST_MM) continue;

            double tx = f.getTargetXDegrees(), ty = f.getTargetYDegrees();
            bearing[i] = h + panR - Math.toRadians(tx);
            horiz[i] = range * Math.cos(Math.toRadians(CAM_PITCH_DEG + ty));
            txDeg[i] = tx;
            yawDeg[i] = f.getTargetPoseCameraSpace().getOrientation().getYaw(AngleUnit.DEGREES);
            seen[i] = true;
            cnt++;
        }
        if (cnt == 0) return;

        int first = -1;
        for (int i = 0; i < N; i++) if (seen[i]) { first = i; break; }
        double ref = bearing[first];

        double[] rel = new double[N];
        double sumI = 0, sumRel = 0, sumH = 0;
        StringBuilder tags = new StringBuilder(), txs = new StringBuilder(), yaws = new StringBuilder();
        for (int i = 0; i < N; i++) {
            if (!seen[i]) continue;
            rel[i] = wrap(bearing[i] - ref);
            sumI += i;
            sumRel += rel[i];
            sumH += horiz[i];
            tags.append(FIRST_TAG + i).append(' ');
            txs.append(String.format("%d:%.1f ", FIRST_TAG + i, txDeg[i]));
            yaws.append(String.format("%d:%.0f ", FIRST_TAG + i, yawDeg[i]));
        }
        double im = sumI / cnt, rm = sumRel / cnt, hm = sumH / cnt;

        lastSeenMs = now;
        lastCnt = cnt;
        lastTags = tags.toString();
        lastTx = txs.toString();
        lastYaw = yaws.toString();

        double aC;
        double c;

        if (cnt >= 2) {
            double num = 0, den = 0, numH = 0;
            for (int i = 0; i < N; i++) {
                if (!seen[i]) continue;
                num += (i - im) * (rel[i] - rm);
                numH += (i - im) * (horiz[i] - hm);
                den += (i - im) * (i - im);
            }
            double d = num / den;
            c = clamp(-d * hm / TAG_SPACING_MM, -1, 1);

            double sumC = 0;
            for (int i = 0; i < N; i++) if (seen[i]) sumC += rel[i] - (i - 1.5) * d;
            aC = ref + sumC / cnt;

            slopeAcc = SLOPE_DECAY * slopeAcc + numH / den;
            updateRowDir(c, aC);
            lastNote = "multi";
        } else {
            if (!haveU) {
                ux = Math.sin(bearing[first]);
                uy = -Math.cos(bearing[first]);
                haveU = true;
                uAssumed = true;
            }
            double beta = bearing[first];
            aC = beta;
            c = 0;
            for (int it = 0; it < 2; it++) {
                c = ux * Math.sin(beta) - uy * Math.cos(beta);
                double d = -TAG_SPACING_MM * c / horiz[first];
                aC = bearing[first] - (first - 1.5) * d;
                beta = aC;
            }
            lastNote = uAssumed ? "single, dir ASSUMED" : "single, dir from state";
        }
        lastC = c;

        double fwd = ux * Math.cos(aC) + uy * Math.sin(aC);
        double sumRc = 0;
        for (int i = 0; i < N; i++) if (seen[i]) sumRc += horiz[i] - (i - 1.5) * TAG_SPACING_MM * fwd;
        double Rc = Math.max(sumRc / cnt, MIN_DIST_MM);
        lastRange = Rc;

        double mx = hX[iPose] + MOUNT_X_MM * Math.cos(h) - MOUNT_Y_MM * Math.sin(h);
        double my = hY[iPose] + MOUNT_X_MM * Math.sin(h) + MOUNT_Y_MM * Math.cos(h);
        double camX = mx + CAM_FWD_MM * Math.cos(h + panR);
        double camY = my + CAM_FWD_MM * Math.sin(h + panR);
        double ax = camX + Rc * Math.cos(aC);
        double ay = camY + Rc * Math.sin(aC);

        if (!haveCenter || Math.hypot(ax - cx, ay - cy) > JUMP_RESET_MM) {
            cx = ax;
            cy = ay;
            if (haveCenter) uAssumed = true;
            haveCenter = true;
        } else {
            cx += CENTER_ALPHA * (ax - cx);
            cy += CENTER_ALPHA * (ay - cy);
        }
    }

    private void updateRowDir(double c, double beta) {
        double s = Math.sqrt(Math.max(0, 1 - c * c));
        double rX = Math.sin(beta), rY = -Math.cos(beta);
        double fX = Math.cos(beta), fY = Math.sin(beta);
        double pX = c * rX + s * fX, pY = c * rY + s * fY;
        double mX = c * rX - s * fX, mY = c * rY - s * fY;

        if (!haveU || uAssumed) {
            if (slopeAcc >= 0) { ux = pX; uy = pY; } else { ux = mX; uy = mY; }
            haveU = true;
            uAssumed = s < SIGN_CONF_SIN;
            return;
        }
        boolean plus = (pX * ux + pY * uy) >= (mX * ux + mY * uy);
        double nx = plus ? pX : mX, ny = plus ? pY : mY;
        ux += DIR_ALPHA * (nx - ux);
        uy += DIR_ALPHA * (ny - uy);
        double n = Math.hypot(ux, uy);
        ux /= n;
        uy /= n;
    }

    // ===================== MISC =====================

    private void drive() {
        double forward = gamepad1.right_stick_x;
        double turn = -gamepad1.left_stick_y;
        stanga.setPower(forward + turn);
        dreapta.setPower(forward - turn);
    }

    private void record(double t, double x, double y, double h, double panDeg) {
        hT[hIdx] = t; hX[hIdx] = x; hY[hIdx] = y; hH[hIdx] = h; hPan[hIdx] = panDeg;
        hIdx = (hIdx + 1) % HIST;
        if (hCount < HIST) hCount++;
    }

    private int lookup(double tq) {
        int best = -1;
        for (int i = 0; i < hCount; i++) {
            int j = (hIdx - 1 - i + HIST) % HIST;
            best = j;
            if (hT[j] <= tq) break;
        }
        return best;
    }

    private static double wrap(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    @Override
    public void stop() {
        if (turret != null) turret.setPower(0);
        if (dashboard != null) dashboard.stopCameraStream();
        if (limelight != null) limelight.stop();
    }
}