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
 * PAN-ONLY cluster tracker (Limelight 3A). Tags FIRST_TAG..FIRST_TAG+3 left -> right in a rigid row.
 *
 * Cluster state (odometry frame): center (cx,cy) + row unit vector u (left -> right).
 * Tag i = center + (i - 1.5) * spacing * u.
 *
 * MULTI-TAG frame (2+ tags):
 *   - angular step d per tag index = least-squares slope of tag bearings (angles only, no range noise)
 *   - c = u . right_of_LOS = -d * range / spacing   (view geometry; |c| = cos of angle between row and image plane)
 *   - u rebuilt from c (two mirror solutions), mirror picked by continuity with stored u;
 *     on first lock / low confidence by range-slope evidence (right-side tags farther or nearer)
 *   - center bearing = mean over tags of (bearing_i - (i-1.5) * d)
 * SINGLE-TAG frame:
 *   - u comes from STATE (fixed in odometry frame), so c, d are predicted for the CURRENT robot position
 *     (no stale value when robot moves) -> center = tag - offset along the row. Works with 1 tag.
 * Center range corrected for each tag's depth offset along the row (uses u).
 */
@Config
@TeleOp(name = "Limelight Cluster Pan", group = "cristi")
public class LimelightClusterPan extends OpMode {

    // ================= CALIBRATION =================
    public static double PAN_CENTER = 0.5;
    public static double PAN_DIR = -1;
    public static double SERVO_RANGE_DEG = 270;
    public static double PAN_MIN_DEG = -120, PAN_MAX_DEG = 120;

    public static double MOUNT_X_MM = 0, MOUNT_Y_MM = 0;
    public static double CAM_FWD_MM = 30;
    public static double CAM_PITCH_DEG = 0;         // fixed camera pitch (tilt servo removed)

    // ================= CLUSTER =================
    public static int FIRST_TAG = 38;               // leftmost tag; 38 39 40 41 left -> right
    public static double TAG_SPACING_MM = 150;      // MEASURE: center-to-center, adjacent tags (now critical)
    public static double DIR_ALPHA = 0.2;           // row direction smoothing
    public static double SIGN_CONF_SIN = 0.5;       // below this |sin(view angle)| mirror sign is not trusted yet
    public static double SLOPE_DECAY = 0.9;         // decay of range-slope evidence

    // ================= TRACKING =================
    public static double tiltPos = 0.2;
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
    private boolean haveCenter = false, haveU = false, uAssumed = false;
    private double cx, cy;                      // cluster center (odometry frame, mm)
    private double ux = 0, uy = 1;              // row unit vector, left -> right
    private double slopeAcc = 0;                // evidence: + means higher tag index is FARTHER

    private double sentPanDeg = 0, reqPan = 0;
    private double lastStaleness = Double.MAX_VALUE;
    private double lastSeenMs = -1e9;
    private double lastRange, lastC;
    private int lastCnt = 0;
    private String lastTags = "-", lastTx = "-", lastYaw = "-", lastNote = "";

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

        pan = hardwareMap.get(Servo.class, "pan");

        tilt = hardwareMap.get(Servo.class, "tilt");
        tilt.setPosition(tiltPos);
        sendPan(0, true);

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
        tilt.setPosition(tiltPos);
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

        // ---------- AIM (pan only) every loop from odometry ----------
        double panDeg = sentPanDeg;
        if (haveCenter) {
            double hAhead = rh + omega * LOOKAHEAD_MS / 1000.0;
            double mx = rx + MOUNT_X_MM * Math.cos(rh) - MOUNT_Y_MM * Math.sin(rh);
            double my = ry + MOUNT_X_MM * Math.sin(rh) + MOUNT_Y_MM * Math.cos(rh);
            panDeg = Math.toDegrees(wrap(Math.atan2(cy - my, cx - mx) - hAhead));
        }
        reqPan = panDeg;
        sendPan(panDeg, false);
        record(now, rx, ry, rh, sentPanDeg);

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
        telemetry.addData("Pan (deg) sent / req", "%.2f / %.1f", sentPanDeg, reqPan);
        telemetry.addData("SATURATED", (reqPan < PAN_MIN_DEG || reqPan > PAN_MAX_DEG) ? "PAN" : "");
        telemetry.addData("Robot", "x %.0f  y %.0f  h %.1f", rx, ry, Math.toDegrees(rh));
        telemetry.addData("Staleness", "%.0f ms", staleness);
        telemetry.addData("Loop", "%.1f ms", loopMs);
        telemetry.update();
    }

    private void processFrame(LLResult r, int iPose, int iServo, double now) {
        double h = hH[iPose];
        double panR = Math.toRadians(hPan[iServo]);

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
            bearing[i] = h + panR - Math.toRadians(tx);          // tx is + to the right
            horiz[i] = range * Math.cos(Math.toRadians(CAM_PITCH_DEG + ty));
            txDeg[i] = tx;
            yawDeg[i] = f.getTargetPoseCameraSpace().getOrientation().getYaw(AngleUnit.DEGREES);
            seen[i] = true;
            cnt++;
        }
        if (cnt == 0) return;

        // ---- stats over seen tags (angles relative to first seen -> no wrap issues) ----
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

        double aC;      // bearing to cluster center (field frame)
        double c;       // u . right-of-LOS

        if (cnt >= 2) {
            // ---------- MULTI-TAG: measure ----------
            double num = 0, den = 0, numH = 0;
            for (int i = 0; i < N; i++) {
                if (!seen[i]) continue;
                num += (i - im) * (rel[i] - rm);
                numH += (i - im) * (horiz[i] - hm);
                den += (i - im) * (i - im);
            }
            double d = num / den;                                   // rad per tag index (negative when facing)
            c = clamp(-d * hm / TAG_SPACING_MM, -1, 1);

            double sumC = 0;
            for (int i = 0; i < N; i++) if (seen[i]) sumC += rel[i] - (i - 1.5) * d;
            aC = ref + sumC / cnt;

            slopeAcc = SLOPE_DECAY * slopeAcc + numH / den;         // + : higher index farther
            updateRowDir(c, aC);
            lastNote = "multi";
        } else {
            // ---------- SINGLE TAG: predict from stored row direction ----------
            if (!haveU) {                                           // first sight, nothing known: face-on guess
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

        // ---- center range: remove each tag's depth offset along the row ----
        double fwd = ux * Math.cos(aC) + uy * Math.sin(aC);        // u . forward-of-LOS
        double sumRc = 0;
        for (int i = 0; i < N; i++) if (seen[i]) sumRc += horiz[i] - (i - 1.5) * TAG_SPACING_MM * fwd;
        double Rc = Math.max(sumRc / cnt, MIN_DIST_MM);
        lastRange = Rc;

        // ---- camera position at frame time -> center in odometry frame ----
        double mx = hX[iPose] + MOUNT_X_MM * Math.cos(h) - MOUNT_Y_MM * Math.sin(h);
        double my = hY[iPose] + MOUNT_X_MM * Math.sin(h) + MOUNT_Y_MM * Math.cos(h);
        double camX = mx + CAM_FWD_MM * Math.cos(h + panR);
        double camY = my + CAM_FWD_MM * Math.sin(h + panR);
        double ax = camX + Rc * Math.cos(aC);
        double ay = camY + Rc * Math.sin(aC);

        if (!haveCenter || Math.hypot(ax - cx, ay - cy) > JUMP_RESET_MM) {
            cx = ax;
            cy = ay;
            if (haveCenter) uAssumed = true;                        // cluster moved: re-learn row sign
            haveCenter = true;
        } else {
            cx += CENTER_ALPHA * (ax - cx);
            cy += CENTER_ALPHA * (ay - cy);
        }
    }

    /**
     * Rebuild row unit vector from measured c = u . right_of_LOS at LOS bearing beta.
     * u = c * right + eps * sqrt(1-c^2) * forward ; eps = +1 if higher tag index is farther.
     * eps: from range-slope evidence while low-confidence, else by continuity with stored u.
     */
    private void updateRowDir(double c, double beta) {
        double s = Math.sqrt(Math.max(0, 1 - c * c));
        double rX = Math.sin(beta), rY = -Math.cos(beta);
        double fX = Math.cos(beta), fY = Math.sin(beta);
        double pX = c * rX + s * fX, pY = c * rY + s * fY;          // eps = +1
        double mX = c * rX - s * fX, mY = c * rY - s * fY;          // eps = -1

        if (!haveU || uAssumed) {
            if (slopeAcc >= 0) { ux = pX; uy = pY; } else { ux = mX; uy = mY; }
            haveU = true;
            uAssumed = s < SIGN_CONF_SIN;                           // stay "assumed" until view is oblique enough
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

    private void sendPan(double panDeg, boolean force) {
        panDeg = clamp(panDeg, PAN_MIN_DEG, PAN_MAX_DEG);
        if (force || Math.abs(panDeg - sentPanDeg) > SERVO_DEADBAND_DEG) {
            pan.setPosition(clamp(PAN_CENTER + PAN_DIR * panDeg / SERVO_RANGE_DEG, 0, 1));
            sentPanDeg = panDeg;
        }
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