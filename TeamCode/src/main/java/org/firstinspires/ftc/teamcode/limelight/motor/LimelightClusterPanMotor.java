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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

//@Config
//@TeleOp(name = "cluster tracker v1", group = "turela")
public class LimelightClusterPanMotor extends OpMode {

    // ================= TURRET MOTOR CALIBRATION =================
    public static double MOTOR_TICKS_PER_REV = 537.7;   // 5203 19.2:1 gearbox. Change if your gearbox differs
    public static double TURRET_RATIO = 3.0;            // motor revs per ONE turret rev (3:1 external)
    public static double TURRET_DIR = 1;                // flip to -1 if +angle reads clockwise
    public static double PAN_MIN_DEG = -120, PAN_MAX_DEG = 120;   // soft limits (cable safe)

    // ================= CONTROLLER =================
    public static boolean TRACK_ENABLED = true;         // dashboard kill switch (power 0 when false)
    public static double kP = 0.02;                     // power per deg of error
    public static double kD = 0.0005;                   // power per deg/s of rate error
    public static double kS = 0.05;                     // static friction boost
    public static double POS_DEADBAND_DEG = 0.5;        // no kS or P inside this (~gear backlash)
    public static double MAX_POWER = 0.6;               // raise after it behaves
    public static double MAX_TURRET_DPS = 550;          // turret speed at power 1.0 (measure, used for feedforward)
    public static double FF_SCALE = 0.9;                // velocity feedforward (main fix for lag)
    public static double RATE_ALPHA = 0.5;              // target-rate low-pass
    public static double JOG_DPS = 60;                  // dpad manual jog speed

    // ================= CAMERA / MOUNT =================
    public static double MOUNT_X_MM = 0, MOUNT_Y_MM = 0;
    public static double CAM_FWD_MM = 30;
    public static double CAM_PITCH_DEG = 0;             // + = camera tilted up

    // ================= 3D =================
    public static boolean USE_3D_BEARING = true;        // bearing from 3D position instead of tx
    public static boolean USE_YAW = true;               // use tag 3D orientation for row direction
    public static double YAW_SIGN = 1;                  // flip if "Row dir yaw" is mirrored vs "multi"
    public static double YAW_OFFSET_DEG = 0;            // raw yaw when face-on (0 or 180 usually)
    public static double YAW_BLEND = 0.5;               // 2+ tags: 0 = angles only, 1 = yaw only
    public static double YAW_MAX_DIST_MM = 3500;        // ignore yaw beyond this (too noisy)

    // ================= CLUSTER =================
    public static int FIRST_TAG = 38;
    public static double TAG_SPACING_MM = 150;
    public static double DIR_ALPHA = 0.2;
    public static double SIGN_CONF_SIN = 0.5;
    public static double SLOPE_DECAY = 0.9;

    // ================= TRACKING =================
    public static int PIPELINE = 2;
    public static double CENTER_ALPHA = 0.5;            // alpha-beta filter: position gain per frame
    public static double CENTER_BETA = 0.15;            // alpha-beta filter: velocity gain per frame
    public static double MAX_TARGET_SPEED_MMS = 2500;   // clamp on estimated tag speed
    public static double VEL_DECAY_MS = 300;            // after this long without tags, tag speed -> 0
    public static double JUMP_RESET_MM = 600;
    public static double EXTRA_LATENCY_MS = 10;
    public static double ENC_LAG_MS = 0;                // encoder is real position, so ~0
    public static double LOOKAHEAD_MS = 30;
    public static double MIN_DIST_MM = 200, MAX_DIST_MM = 6000;

    // ================= HARDWARE =================
    private DcMotor stanga, dreapta;
    private DcMotorEx turret;
    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;
    private FtcDashboard dashboard;

    // ================= STATE =================
    private static final int N = 4;
    private boolean haveCenter = false, haveU = false, uAssumed = false;
    private double cx, cy;                      // cluster center at time cT (odometry frame, mm)
    private double cvx = 0, cvy = 0, cT = 0;    // cluster center velocity (mm/s), state time (ms)
    private double ux = 0, uy = 1;
    private double slopeAcc = 0;

    private double zeroTicks = 0;
    private double turretDeg = 0, turretVel = 0;       // measured, deg and deg/s
    private double targetDeg = 0, lastTargetDeg = 0, targetRate = 0;
    private double lastPower = 0;
    private boolean lastB = false;

    private double lastStaleness = Double.MAX_VALUE;
    private double lastSeenMs = -1e9;
    private double lastRange, lastC;
    private int lastCnt = 0;
    private String lastTags = "-", lastTx = "-", lastYaw = "-", lastNote = "", lastHoriz = "-";
    private double lastRowYawDeg = Double.NaN, lastRowMultiDeg = Double.NaN;

    private final ElapsedTime runtime = new ElapsedTime();
    private double lastLoopMs = 0;

    private static final int HIST = 256;
    private final double[] hT = new double[HIST], hX = new double[HIST], hY = new double[HIST],
            hH = new double[HIST], hPan = new double[HIST];
    private int hIdx = 0, hCount = 0;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);    // power control, our own loop
        zeroTicks = 0;

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
    }

    @Override
    public void start() {
        runtime.reset();
        lastLoopMs = 0;
    }

    private double ticksPerDeg() {
        return MOTOR_TICKS_PER_REV * TURRET_RATIO / 360.0;
    }

    @Override
    public void loop() {
        double now = runtime.milliseconds();
        double loopMs = now - lastLoopMs;
        lastLoopMs = now;
        double dt = Math.max(loopMs, 1) / 1000.0;
//        drive();

        // ---------- TURRET MEASUREMENT ----------
        if (gamepad1.b && !lastB) zeroTicks = turret.getCurrentPosition();   // re-zero (turret centered)
        lastB = gamepad1.b;
        double tpd = ticksPerDeg();
        turretDeg = TURRET_DIR * (turret.getCurrentPosition() - zeroTicks) / tpd;
        turretVel = TURRET_DIR * turret.getVelocity() / tpd;

        // ---------- ODOMETRY ----------
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        double rx = pose.getX(DistanceUnit.MM);
        double ry = pose.getY(DistanceUnit.MM);
        double rh = pose.getHeading(AngleUnit.RADIANS);
        double omega = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

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
                int iTurret = lookup(now - agoMs - ENC_LAG_MS);
                if (iPose >= 0 && iTurret >= 0) processFrame(r, iPose, iTurret, now, now - agoMs);
            }
        }

        // ---------- AIM (every loop from odometry) ----------
        boolean jog = gamepad1.dpad_left || gamepad1.dpad_right;
        if (jog) {
            double dir = gamepad1.dpad_left ? 1 : -1;       // left = CCW = +
            targetDeg = clamp(lastTargetDeg + dir * JOG_DPS * dt, PAN_MIN_DEG, PAN_MAX_DEG);
            targetRate = 0;
        } else if (haveCenter) {
            // no tags for a while -> stop extrapolating tag motion (hold last point)
            if (now - lastSeenMs > VEL_DECAY_MS) {
                cx += cvx * (now - cT) / 1000.0;
                cy += cvy * (now - cT) / 1000.0;
                cvx = 0;
                cvy = 0;
                cT = now;
            }
            // center predicted to NOW + lookahead (moves smoothly between camera frames)
            double tA = (now + LOOKAHEAD_MS - cT) / 1000.0;
            double tgx = cx + cvx * tA, tgy = cy + cvy * tA;
            double hAhead = rh + omega * LOOKAHEAD_MS / 1000.0;
            double mx = rx + MOUNT_X_MM * Math.cos(rh) - MOUNT_Y_MM * Math.sin(rh);
            double my = ry + MOUNT_X_MM * Math.sin(rh) + MOUNT_Y_MM * Math.cos(rh);
            double dx = tgx - mx, dy = tgy - my;
            double desired = Math.toDegrees(wrap(Math.atan2(dy, dx) - hAhead));
            targetDeg = fitToLimits(desired, turretDeg);

            // analytic target rate (no differentiation noise): tag motion - robot motion - robot spin
            double rvx = cvx - pinpoint.getVelX(DistanceUnit.MM);
            double rvy = cvy - pinpoint.getVelY(DistanceUnit.MM);
            double d2 = Math.max(dx * dx + dy * dy, 1);
            double rawRate = Math.toDegrees((dx * rvy - dy * rvx) / d2 - omega);
            targetRate += RATE_ALPHA * (rawRate - targetRate);
        } else {
            targetDeg = lastTargetDeg;                      // hold until first sight
            targetRate = 0;
        }
        lastTargetDeg = targetDeg;

        runTurret(targetDeg, targetRate);
        record(now, rx, ry, rh, turretDeg);

        // ---------- TELEMETRY ----------
        double sinceSeen = (now - lastSeenMs) / 1000.0;
        telemetry.addData("Mode", jog ? "MANUAL JOG" : !haveCenter ? "NO TAG YET"
                                                       : sinceSeen < 0.2 ? "VISION LOCK" : String.format("ODOMETRY ONLY (%.1fs)", sinceSeen));
        telemetry.addData("Tags / count", "%s / %d  (%s)", lastTags, lastCnt, lastNote);
        telemetry.addData("Horiz dist cam->tag (mm)", lastHoriz);
        telemetry.addData("Horiz dist cam->center (mm)", "%.0f", lastRange);
        telemetry.addData("Per-tag tx | 3D bearing (deg)", lastTx);
        telemetry.addData("Yaw raw | corrected (deg)", lastYaw);
        telemetry.addData("Row dir", !haveU ? "none" : String.format("%.1f deg  %s",
                Math.toDegrees(Math.atan2(uy, ux)), uAssumed ? "ASSUMED/low-conf" : "measured"));
        telemetry.addData("Row dir yaw / multi", "%.1f / %.1f", lastRowYawDeg, lastRowMultiDeg);
        telemetry.addData("View c / angle", "%.2f / %.0f deg  (cfg spacing %.0f)",
                lastC, Math.toDegrees(Math.acos(clamp(lastC, -1, 1))), TAG_SPACING_MM);
        telemetry.addData("Center (mm) / tag speed (mm/s)", "x %.0f  y %.0f  /  %.0f", cx, cy, Math.hypot(cvx, cvy));
        // Graph these three on the dashboard while tuning
        telemetry.addData("Turret target", "%.2f", targetDeg);
        telemetry.addData("Turret actual", "%.2f", turretDeg);
        telemetry.addData("Turret error", "%.2f", targetDeg - turretDeg);
        telemetry.addData("Turret vel / tgt rate (dps)", "%.1f / %.1f", turretVel, targetRate);
        telemetry.addData("Power", "%.2f", lastPower);
        telemetry.addData("Raw ticks / ticksPerDeg", "%d / %.3f", turret.getCurrentPosition(), tpd);
        telemetry.addData("AT LIMIT", (turretDeg <= PAN_MIN_DEG || turretDeg >= PAN_MAX_DEG) ? "YES" : "");
        telemetry.addData("Robot", "x %.0f  y %.0f  h %.1f", rx, ry, Math.toDegrees(rh));
        telemetry.addData("Staleness", "%.0f ms", staleness);
        telemetry.addData("Loop", "%.1f ms", loopMs);
        telemetry.update();
    }

    /** Position loop on the turret motor. */
    private void runTurret(double target, double tgtRate) {
        double u = 0;
        if (TRACK_ENABLED) {
            double err = target - turretDeg;
            if (Math.abs(err) > POS_DEADBAND_DEG) {
                u = kP * err + kS * Math.signum(err);
            }
            u += kD * (tgtRate - turretVel);
            u += FF_SCALE * tgtRate / MAX_TURRET_DPS;
            u = clamp(u, -MAX_POWER, MAX_POWER);

            // hard stop: never push further out of the limits
            if (turretDeg >= PAN_MAX_DEG && u > 0) u = 0;
            if (turretDeg <= PAN_MIN_DEG && u < 0) u = 0;
        }
        lastPower = u;
        double p = u * TURRET_DIR;      // convert angle-sense power back to motor sense
        turret.setPower(p);
    }

    /** Pick the equivalent of desired (+/- 360k) that is inside limits and closest to current. */
    private double fitToLimits(double desired, double current) {
        double best = Double.NaN;
        for (int k = -1; k <= 1; k++) {
            double c = desired + 360.0 * k;
            if (c < PAN_MIN_DEG || c > PAN_MAX_DEG) continue;
            if (Double.isNaN(best) || Math.abs(c - current) < Math.abs(best - current)) best = c;
        }
        if (Double.isNaN(best)) best = clamp(desired, PAN_MIN_DEG, PAN_MAX_DEG);
        return best;
    }

    private void processFrame(LLResult r, int iPose, int iTurret, double now, double frameT) {
        double h = hH[iPose];
        double panR = Math.toRadians(hPan[iTurret]);

        boolean[] seen = new boolean[N];
        double[] bearing = new double[N], horiz = new double[N], txDeg = new double[N], yawDeg = new double[N],
                b3Deg = new double[N];
        int cnt = 0;

        double camHead = h + panR;
        double cp = Math.cos(Math.toRadians(CAM_PITCH_DEG)), sp = Math.sin(Math.toRadians(CAM_PITCH_DEG));
        double yx = 0, yy = 0, ySum = 0;            // weighted row direction from 3D yaw

        for (LLResultTypes.FiducialResult f : r.getFiducialResults()) {
            int i = f.getFiducialId() - FIRST_TAG;
            if (i < 0 || i >= N || seen[i]) continue;

            Position p = f.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.MM);
            double range = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            if (range <= MIN_DIST_MM || range >= MAX_DIST_MM) continue;

            // camera space: x right, y down, z forward. Level it by the camera pitch.
            double fwdH = p.z * cp + p.y * sp;                  // horizontal forward component
            double hor = Math.hypot(p.x, fwdH);                 // HORIZONTAL distance camera -> tag
            double b3 = Math.atan2(p.x, fwdH);                  // + = tag to the right
            double tx = f.getTargetXDegrees();

            bearing[i] = camHead - (USE_3D_BEARING ? b3 : Math.toRadians(tx));
            horiz[i] = hor;
            txDeg[i] = tx;
            b3Deg[i] = Math.toDegrees(b3);
            yawDeg[i] = f.getTargetPoseCameraSpace().getOrientation().getYaw(AngleUnit.DEGREES);
            seen[i] = true;
            cnt++;

            // row direction from this tag's 3D orientation: face-on -> row = camera right
            if (hor < YAW_MAX_DIST_MM) {
                double yawC = Math.toRadians(YAW_SIGN * wrapDeg(yawDeg[i] - YAW_OFFSET_DEG));
                double rowAng = camHead - Math.PI / 2 + yawC;
                double w = 1.0 / Math.pow(Math.max(hor / 1000.0, 0.3), 2);   // closer = trusted more
                yx += w * Math.cos(rowAng);
                yy += w * Math.sin(rowAng);
                ySum += w;
            }
        }
        if (cnt == 0) return;

        boolean yawOK = USE_YAW && ySum > 0 && Math.hypot(yx, yy) > 1e-6;
        double yux = 0, yuy = 0;
        if (yawOK) {
            double n = Math.hypot(yx, yy);
            yux = yx / n;
            yuy = yy / n;
            lastRowYawDeg = Math.toDegrees(Math.atan2(yuy, yux));
        } else {
            lastRowYawDeg = Double.NaN;
        }

        int first = -1;
        for (int i = 0; i < N; i++) if (seen[i]) { first = i; break; }
        double ref = bearing[first];

        double[] rel = new double[N];
        double sumI = 0, sumRel = 0, sumH = 0;
        StringBuilder tags = new StringBuilder(), txs = new StringBuilder(), yaws = new StringBuilder(),
                hs = new StringBuilder();
        for (int i = 0; i < N; i++) {
            if (!seen[i]) continue;
            rel[i] = wrap(bearing[i] - ref);
            sumI += i;
            sumRel += rel[i];
            sumH += horiz[i];
            tags.append(FIRST_TAG + i).append(' ');
            txs.append(String.format("%d:%.1f|%.1f ", FIRST_TAG + i, txDeg[i], b3Deg[i]));
            yaws.append(String.format("%d:%.0f|%.0f ", FIRST_TAG + i, yawDeg[i],
                    YAW_SIGN * wrapDeg(yawDeg[i] - YAW_OFFSET_DEG)));
            hs.append(String.format("%d:%.0f ", FIRST_TAG + i, horiz[i]));
        }
        double im = sumI / cnt, rm = sumRel / cnt, hm = sumH / cnt;

        lastSeenMs = now;
        lastCnt = cnt;
        lastTags = tags.toString();
        lastTx = txs.toString();
        lastYaw = yaws.toString();
        lastHoriz = hs.toString();

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
            updateRowDir(c, aC, yawOK, yux, yuy);
            lastNote = yawOK ? "multi + 3D yaw" : "multi";
        } else {
            lastRowMultiDeg = Double.NaN;
            if (yawOK) {                                        // single tag: full direction from 3D yaw
                if (!haveU || uAssumed) {
                    ux = yux;
                    uy = yuy;
                } else {
                    ux += DIR_ALPHA * (yux - ux);
                    uy += DIR_ALPHA * (yuy - uy);
                    double n = Math.hypot(ux, uy);
                    ux /= n;
                    uy /= n;
                }
                haveU = true;
                uAssumed = false;
            } else if (!haveU) {
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
            lastNote = yawOK ? "single, dir from 3D yaw"
                    : uAssumed ? "single, dir ASSUMED" : "single, dir from state";
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

        // alpha-beta filter: predict center to frame time, correct position AND velocity
        double dtF = clamp((frameT - cT) / 1000.0, 0.005, 0.5);
        double px = cx + cvx * dtF, py = cy + cvy * dtF;
        double ex = ax - px, ey = ay - py;
        if (!haveCenter || Math.hypot(ex, ey) > JUMP_RESET_MM) {
            cx = ax;
            cy = ay;
            cvx = 0;
            cvy = 0;
            if (haveCenter) uAssumed = true;
            haveCenter = true;
        } else {
            cx = px + CENTER_ALPHA * ex;
            cy = py + CENTER_ALPHA * ey;
            cvx += CENTER_BETA * ex / dtF;
            cvy += CENTER_BETA * ey / dtF;
            double spd = Math.hypot(cvx, cvy);
            if (spd > MAX_TARGET_SPEED_MMS) { cvx *= MAX_TARGET_SPEED_MMS / spd; cvy *= MAX_TARGET_SPEED_MMS / spd; }
        }
        cT = frameT;
    }

    private void updateRowDir(double c, double beta, boolean yawOK, double yux, double yuy) {
        double s = Math.sqrt(Math.max(0, 1 - c * c));
        double rX = Math.sin(beta), rY = -Math.cos(beta);
        double fX = Math.cos(beta), fY = Math.sin(beta);
        double pX = c * rX + s * fX, pY = c * rY + s * fY;
        double mX = c * rX - s * fX, mY = c * rY - s * fY;

        if (yawOK) {
            // 3D yaw picks the mirror solution, then blend angle-based and yaw-based directions
            boolean plus = (pX * yux + pY * yuy) >= (mX * yux + mY * yuy);
            double aX = plus ? pX : mX, aY = plus ? pY : mY;
            lastRowMultiDeg = Math.toDegrees(Math.atan2(aY, aX));
            double nx = (1 - YAW_BLEND) * aX + YAW_BLEND * yux;
            double ny = (1 - YAW_BLEND) * aY + YAW_BLEND * yuy;
            double n = Math.hypot(nx, ny);
            if (n < 1e-6) { nx = yux; ny = yuy; n = 1; }
            nx /= n;
            ny /= n;
            if (!haveU || uAssumed) {
                ux = nx;
                uy = ny;
            } else {
                ux += DIR_ALPHA * (nx - ux);
                uy += DIR_ALPHA * (ny - uy);
                double m = Math.hypot(ux, uy);
                ux /= m;
                uy /= m;
            }
            haveU = true;
            uAssumed = false;
            return;
        }

        lastRowMultiDeg = Math.toDegrees(Math.atan2(slopeAcc >= 0 ? pY : mY, slopeAcc >= 0 ? pX : mX));
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

    private static double wrapDeg(double a) {
        return Math.toDegrees(wrap(Math.toRadians(a)));
    }

    private static double wrap(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    @Override
    public void stop() {
        turret.setPower(0);
        dashboard.stopCameraStream();
        limelight.stop();
    }
}