package org.firstinspires.ftc.teamcode;

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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/**
 * Rigid-row model. Tags FIRST_TAG..FIRST_TAG+3 (38 39 40 41, left -> right) sit in a line.
 * Tag i is at (i - 1.5) * spacing along the row from the hive center.
 * ANY visible tag -> refreshes the hive center directly (no stale estimates).
 * Row direction is measured whenever 2+ tags are seen in the same frame.
 */
@Config
@TeleOp(name = "Limelight Cluster Tracker", group = "cristi")
public class Limelightclustertracker extends OpMode {

    // ================= CALIBRATION (measure these) =================
    public static double PAN_CENTER = 0.5;
    public static double TILT_CENTER = 0.5;
    public static double PAN_DIR = -1;
    public static double TILT_DIR = 1;
    public static double SERVO_RANGE_DEG = 270;

    public static double PAN_MIN_DEG = -120, PAN_MAX_DEG = 120;
    public static double TILT_MIN_DEG = -15, TILT_MAX_DEG = 40;

    public static double MOUNT_X_MM = 0, MOUNT_Y_MM = 0;
    public static double CAM_FWD_MM = 30;

    // ================= HIVE ROW =================
    public static int FIRST_TAG = 38;               // leftmost; row is FIRST_TAG .. FIRST_TAG+3, left -> right
    // Center-to-center distance between ADJACENT tags (mm). MEASURE IT (placeholder!).
    // Telemetry "Measured spacing" shows a live estimate when 2+ tags are visible.
    public static double TAG_SPACING_MM = 150;
    public static double DIR_ALPHA = 0.2;           // row direction smoothing

    // Closed-loop trim: when a SYMMETRIC pair is visible (38+41 or 39+40), image midpoint -> crosshair
    public static boolean CLOSED_LOOP = true;
    public static double K_PAN = 0.4, K_TILT = 0.4;
    public static double CORR_MAX_DEG = 15;

    // ================= TRACKING =================
    public static int PIPELINE = 2;

    public static double CENTER_ALPHA = 0.3;
    public static double JUMP_RESET_MM = 400;
    public static double EXTRA_LATENCY_MS = 10;
    public static double SERVO_LAG_MS = 30;
    public static double LOOKAHEAD_MS = 30;
    public static double SERVO_DEADBAND_DEG = 0.15;
    public static double MIN_DIST_MM = 200, MAX_DIST_MM = 6000;

    // ================= HARDWARE =================
    private DcMotor stanga, dreapta;
    private Limelight3A limelight;
    private Servo pan, tilt;
    private GoBildaPinpointDriver pinpoint;
    private FtcDashboard dashboard;

    // ================= STATE =================
    private static final int N = 4;
    private boolean haveCenter = false, haveDir = false;
    private double cx, cy, cz;                  // hive center (odometry frame, mm)
    private double ux = 0, uy = 1;              // row unit vector, left -> right

    private double sentPanDeg = 0, sentTiltDeg = 0;
    private double reqPan = 0, reqTilt = 0;
    private double panCorr = 0, tiltCorr = 0;
    private double lastStaleness = Double.MAX_VALUE;
    private double lastSeenMs = -1e9;
    private double lastTx, lastTy, lastRange, lastSpacing;
    private String lastSeenTags = "-";

    private final ElapsedTime runtime = new ElapsedTime();
    private double lastLoopMs = 0;

    private static final int HIST = 256;
    private final double[] hT = new double[HIST], hX = new double[HIST], hY = new double[HIST],
            hH = new double[HIST], hPan = new double[HIST], hTilt = new double[HIST];
    private int hIdx = 0, hCount = 0;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        pan = hardwareMap.get(Servo.class, "pan");
        tilt = hardwareMap.get(Servo.class, "tilt");
        sendServos(0, 0, true);

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

    @Override
    public void loop() {
        double now = runtime.milliseconds();
        double loopMs = now - lastLoopMs;
        lastLoopMs = now;

        drive();

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
                int iServo = lookup(now - agoMs - SERVO_LAG_MS);
                if (iPose >= 0 && iServo >= 0) processFrame(r, iPose, iServo, now);
            }
        }

        // ---------- AIM every loop from odometry ----------
        double panDeg = sentPanDeg, tiltDeg = sentTiltDeg;
        if (haveCenter) {
            double hAhead = rh + omega * LOOKAHEAD_MS / 1000.0;
            double mx = rx + MOUNT_X_MM * Math.cos(rh) - MOUNT_Y_MM * Math.sin(rh);
            double my = ry + MOUNT_X_MM * Math.sin(rh) + MOUNT_Y_MM * Math.cos(rh);
            double dx = cx - mx, dy = cy - my;

            panDeg = Math.toDegrees(wrap(Math.atan2(dy, dx) - hAhead)) + panCorr;
            double horiz = Math.max(Math.hypot(dx, dy) - CAM_FWD_MM, 1);
            tiltDeg = Math.toDegrees(Math.atan2(cz, horiz)) + tiltCorr;
        }
        reqPan = panDeg;
        reqTilt = tiltDeg;
        sendServos(panDeg, tiltDeg, false);
        record(now, rx, ry, rh, sentPanDeg, sentTiltDeg);

        // ---------- TELEMETRY ----------
        double sinceSeen = (now - lastSeenMs) / 1000.0;
        telemetry.addData("Mode", !haveCenter ? "NO TAG YET"
                : sinceSeen < 0.2 ? "VISION LOCK" : String.format("ODOMETRY ONLY (%.1fs)", sinceSeen));
        telemetry.addData("Tags in last frame", lastSeenTags);
        telemetry.addData("Row dir", haveDir ? "measured" : "guessed (need 2 tags once)");
        telemetry.addData("Measured spacing (mm)", "%.0f  (config %.0f)", lastSpacing, TAG_SPACING_MM);
        telemetry.addData("tx / ty / range (avg)", "%.2f / %.2f / %.0f mm", lastTx, lastTy, lastRange);
        telemetry.addData("Hive center (mm)", "x %.0f  y %.0f  dz %.0f", cx, cy, cz);
        telemetry.addData("Pan / Tilt (deg)", "%.2f / %.2f", sentPanDeg, sentTiltDeg);
        telemetry.addData("Requested (deg)", "%.1f / %.1f", reqPan, reqTilt);
        telemetry.addData("Trim (deg)", "%.2f / %.2f", panCorr, tiltCorr);
        telemetry.addData("SATURATED", (reqPan < PAN_MIN_DEG || reqPan > PAN_MAX_DEG ? "PAN " : "")
                + (reqTilt < TILT_MIN_DEG || reqTilt > TILT_MAX_DEG ? "TILT" : ""));
        telemetry.addData("Robot", "x %.0f  y %.0f  h %.1f", rx, ry, Math.toDegrees(rh));
        telemetry.addData("Staleness", "%.0f ms", staleness);
        telemetry.addData("Loop", "%.1f ms", loopMs);
        telemetry.update();
    }

    /** Measure every visible hive tag, update row direction, then refresh the hive center. */
    private void processFrame(LLResult r, int iPose, int iServo, double now) {
        boolean[] seen = new boolean[N];
        double[] mX = new double[N], mY = new double[N], mZ = new double[N];
        double sTx = 0, sTy = 0, sRange = 0;

        for (LLResultTypes.FiducialResult f : r.getFiducialResults()) {
            int i = f.getFiducialId() - FIRST_TAG;
            if (i < 0 || i >= N) continue; // not part of this hive

            Position p = f.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.MM);
            double range = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            if (range <= MIN_DIST_MM || range >= MAX_DIST_MM) continue;

            double[] m = measure(iPose, iServo, f.getTargetXDegrees(), f.getTargetYDegrees(), range);
            seen[i] = true;
            mX[i] = m[0];
            mY[i] = m[1];
            mZ[i] = m[2];
            sTx += f.getTargetXDegrees();
            sTy += f.getTargetYDegrees();
            sRange += range;
        }

        int cnt = 0, lo = -1, hi = -1;
        double sumS = 0;
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < N; i++) {
            if (!seen[i]) continue;
            cnt++;
            sumS += i - 1.5;
            if (lo < 0) lo = i;
            hi = i;
            sb.append(FIRST_TAG + i).append(' ');
        }
        if (cnt == 0) return;

        lastSeenMs = now;
        lastSeenTags = sb.toString();
        lastTx = sTx / cnt;
        lastTy = sTy / cnt;
        lastRange = sRange / cnt;

        // --- row direction (left -> right) ---
        if (cnt >= 2) {
            double dx = mX[hi] - mX[lo], dy = mY[hi] - mY[lo];
            double len = Math.hypot(dx, dy);
            if (len > 100) {
                double nx = dx / len, ny = dy / len;
                if (!haveDir) {
                    ux = nx;
                    uy = ny;
                    haveDir = true;
                } else {
                    ux += DIR_ALPHA * (nx - ux);
                    uy += DIR_ALPHA * (ny - uy);
                    double n = Math.hypot(ux, uy);
                    ux /= n;
                    uy /= n;
                }
                lastSpacing = len / (hi - lo);
            }
        }
        if (!haveDir) {
            // guess: hive faces the robot -> row perpendicular to line of sight, right = bearing - 90 deg
            int k = lo;
            double bearing = Math.atan2(mY[k] - hY[iPose], mX[k] - hX[iPose]);
            ux = Math.sin(bearing);
            uy = -Math.cos(bearing);
        }

        // --- hive center: each seen tag votes (tag position - its offset along the row) ---
        double ax = 0, ay = 0, az = 0;
        for (int i = 0; i < N; i++) {
            if (!seen[i]) continue;
            double s = (i - 1.5) * TAG_SPACING_MM;
            ax += mX[i] - s * ux;
            ay += mY[i] - s * uy;
            az += mZ[i];
        }
        ax /= cnt;
        ay /= cnt;
        az /= cnt;

        if (!haveCenter || Math.hypot(ax - cx, ay - cy) > JUMP_RESET_MM) {
            cx = ax;
            cy = ay;
            cz = az;
            haveCenter = true;
        } else {
            cx += CENTER_ALPHA * (ax - cx);
            cy += CENTER_ALPHA * (ay - cy);
            cz += CENTER_ALPHA * (az - cz);
        }

        // --- closed-loop trim: only when seen tags are symmetric around the center ---
        if (CLOSED_LOOP && cnt >= 2 && Math.abs(sumS) < 1e-6) {
            // tx>0 = right of crosshair -> pan (left-positive) decreases; ty>0 = above -> tilt increases
            panCorr = clamp(panCorr - K_PAN * lastTx, -CORR_MAX_DEG, CORR_MAX_DEG);
            tiltCorr = clamp(tiltCorr + K_TILT * lastTy, -CORR_MAX_DEG, CORR_MAX_DEG);
        }
    }

    /** Tag position (odometry frame, mm) + height above camera pivot from one detection. */
    private double[] measure(int iPose, int iServo, double tx, double ty, double range) {
        double h = hH[iPose], x = hX[iPose], y = hY[iPose];
        double panR = Math.toRadians(hPan[iServo]);
        double tiltR = Math.toRadians(hTilt[iServo]);

        double bearing = h + panR - Math.toRadians(tx); // tx is + to the right
        double elev = tiltR + Math.toRadians(ty);
        double horiz = range * Math.cos(elev);
        double dz = range * Math.sin(elev);

        double mx = x + MOUNT_X_MM * Math.cos(h) - MOUNT_Y_MM * Math.sin(h);
        double my = y + MOUNT_X_MM * Math.sin(h) + MOUNT_Y_MM * Math.cos(h);
        double camX = mx + CAM_FWD_MM * Math.cos(h + panR);
        double camY = my + CAM_FWD_MM * Math.sin(h + panR);

        return new double[]{camX + horiz * Math.cos(bearing), camY + horiz * Math.sin(bearing), dz};
    }

    private void sendServos(double panDeg, double tiltDeg, boolean force) {
        panDeg = clamp(panDeg, PAN_MIN_DEG, PAN_MAX_DEG);
        tiltDeg = clamp(tiltDeg, TILT_MIN_DEG, TILT_MAX_DEG);
        if (force || Math.abs(panDeg - sentPanDeg) > SERVO_DEADBAND_DEG) {
            pan.setPosition(clamp(PAN_CENTER + PAN_DIR * panDeg / SERVO_RANGE_DEG, 0, 1));
            sentPanDeg = panDeg;
        }
        if (force || Math.abs(tiltDeg - sentTiltDeg) > SERVO_DEADBAND_DEG) {
            tilt.setPosition(clamp(TILT_CENTER + TILT_DIR * tiltDeg / SERVO_RANGE_DEG, 0, 1));
            sentTiltDeg = tiltDeg;
        }
    }

    private void drive() {
        double forward = gamepad1.right_stick_x;
        double turn = -gamepad1.left_stick_y;

        stanga.setPower(forward + turn);
        dreapta.setPower(forward - turn);
    }

    private void record(double t, double x, double y, double h, double panDeg, double tiltDeg) {
        hT[hIdx] = t; hX[hIdx] = x; hY[hIdx] = y; hH[hIdx] = h;
        hPan[hIdx] = panDeg; hTilt[hIdx] = tiltDeg;
        hIdx = (hIdx + 1) % HIST;
        if (hCount < HIST) hCount++;
    }

    /** Index of the newest sample at or before time tq (oldest if none), -1 if empty. */
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
        dashboard.stopCameraStream();
        limelight.stop();
    }
}