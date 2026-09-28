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

@Config
@TeleOp(name = "Limelight Turret Tracker v3", group = "cristi")
public class limelightTracker extends OpMode {

    // ================= CALIBRATION (measure these) =================
    // Servo position where the camera points straight FORWARD / is LEVEL
    public static double PAN_CENTER = 0.5;
    public static double TILT_CENTER = 0.5;
    // Set so that +angle = pan LEFT (counter-clockwise from above) and tilt UP
    public static double PAN_DIR = -1;
    public static double TILT_DIR = 1;
    public static double SERVO_RANGE_DEG = 270; // default SDK PWM on goBILDA dual mode

    // Angle limits in degrees from center (cables/frame)
    public static double PAN_MIN_DEG = -120, PAN_MAX_DEG = 120;
    public static double TILT_MIN_DEG = -15, TILT_MAX_DEG = 40;

    // Pan axis position relative to the Pinpoint tracking point (mm), +X forward, +Y left
    public static double MOUNT_X_MM = 0, MOUNT_Y_MM = 0;
    // Distance of the lens in front of the pan axis (mm)
    public static double CAM_FWD_MM = 30;

    // ================= TRACKING =================
    public static int targetTag = 41;
    public static int PIPELINE = 2;

    public static double TAG_ALPHA = 0.3;          // 0..1, lower = smoother, higher = trusts each frame more
    public static double HEIGHT_ALPHA = 0.3;
    public static double JUMP_RESET_MM = 400;      // re-lock instantly if estimate is off by more than this
    public static double EXTRA_LATENCY_MS = 10;    // USB + loop delay not reported by Limelight
    public static double SERVO_LAG_MS = 30;        // how far the real servo lags the command
    public static double LOOKAHEAD_MS = 30;        // aim ahead of robot rotation
    public static double SERVO_DEADBAND_DEG = 0.15; // don't send tiny changes -> no servo buzz
    public static double MIN_DIST_MM = 200, MAX_DIST_MM = 6000;

    // ================= HARDWARE =================
    private DcMotor stanga, dreapta;
    private Limelight3A limelight;
    private Servo pan, tilt;
    private GoBildaPinpointDriver pinpoint;
    private FtcDashboard dashboard;

    // ================= STATE =================
    private boolean haveTag = false;
    private double tagX, tagY, tagDz;          // tag position in field frame (mm)
    private double sentPanDeg = 0, sentTiltDeg = 0;
    private double lastStaleness = Double.MAX_VALUE;
    private double lastSeenMs = -1e9;
    private double lastTx, lastTy, lastRange;

    private final ElapsedTime runtime = new ElapsedTime();
    private double lastLoopMs = 0;

    // History for latency compensation
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
        // COPY THESE 3 LINES FROM YOUR DRIVE / ODOMETRY CODE
        pinpoint.setOffsets(0, 0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU(); // keep robot still during init

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

        // ---------- VISION: update tag position on new frames ----------
        LLResult r = limelight.getLatestResult();
        double staleness = -1;
        if (r != null) {
            staleness = r.getStaleness();
            boolean newFrame = staleness < lastStaleness;
            lastStaleness = staleness;

            if (newFrame && r.isValid()) {
                for (LLResultTypes.FiducialResult f : r.getFiducialResults()) {
                    if (f.getFiducialId() != targetTag) continue;

                    Position p = f.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.MM);
                    double range = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                    double agoMs = staleness + r.getCaptureLatency() + r.getTargetingLatency()
                            + EXTRA_LATENCY_MS;
                    int iPose = lookup(now - agoMs);
                    int iServo = lookup(now - agoMs - SERVO_LAG_MS);

                    lastTx = f.getTargetXDegrees();
                    lastTy = f.getTargetYDegrees();
                    lastRange = range;

                    if (range > MIN_DIST_MM && range < MAX_DIST_MM && iPose >= 0 && iServo >= 0) {
                        updateTagEstimate(iPose, iServo, lastTx, lastTy, range);
                        lastSeenMs = now;
                    }
                    break;
                }
            }
        }

        // ---------- AIM every loop from odometry ----------
        double panDeg = sentPanDeg, tiltDeg = sentTiltDeg;
        if (haveTag) {
            double hAhead = rh + omega * LOOKAHEAD_MS / 1000.0;
            double mx = rx + MOUNT_X_MM * Math.cos(rh) - MOUNT_Y_MM * Math.sin(rh);
            double my = ry + MOUNT_X_MM * Math.sin(rh) + MOUNT_Y_MM * Math.cos(rh);
            double dx = tagX - mx, dy = tagY - my;

            panDeg = Math.toDegrees(wrap(Math.atan2(dy, dx) - hAhead));
            double horiz = Math.max(Math.hypot(dx, dy) - CAM_FWD_MM, 1);
            tiltDeg = Math.toDegrees(Math.atan2(tagDz, horiz));
        }
        sendServos(panDeg, tiltDeg, false);
        record(now, rx, ry, rh, sentPanDeg, sentTiltDeg);

        // ---------- TELEMETRY ----------
        double sinceSeen = (now - lastSeenMs) / 1000.0;
        telemetry.addData("Mode", !haveTag ? "NO TAG YET"
                : sinceSeen < 0.2 ? "VISION LOCK" : String.format("ODOMETRY ONLY (%.1fs)", sinceSeen));
        telemetry.addData("tx / ty / range", "%.2f / %.2f / %.0f mm", lastTx, lastTy, lastRange);
        telemetry.addData("Tag est (mm)", "x %.0f  y %.0f  dz %.0f", tagX, tagY, tagDz);
        telemetry.addData("Pan / Tilt (deg)", "%.2f / %.2f", sentPanDeg, sentTiltDeg);
        telemetry.addData("Robot", "x %.0f  y %.0f  h %.1f", rx, ry, Math.toDegrees(rh));
        telemetry.addData("Staleness", "%.0f ms", staleness);
        telemetry.addData("Loop", "%.1f ms", loopMs);
        telemetry.update();
    }

    private void updateTagEstimate(int iPose, int iServo, double tx, double ty, double range) {
        double h = hH[iPose], x = hX[iPose], y = hY[iPose];
        double panR = Math.toRadians(hPan[iServo]);
        double tiltR = Math.toRadians(hTilt[iServo]);

        double bearing = h + panR - Math.toRadians(tx); // tx is + to the right
        double elev = tiltR + Math.toRadians(ty);
        double horiz = range * Math.cos(elev);
        double dz = range * Math.sin(elev);

        double mx = x + MOUNT_X_MM * Math.cos(h) - MOUNT_Y_MM * Math.sin(h);
        double my = y + MOUNT_X_MM * Math.sin(h) + MOUNT_Y_MM * Math.cos(h);
        double cx = mx + CAM_FWD_MM * Math.cos(h + panR);
        double cy = my + CAM_FWD_MM * Math.sin(h + panR);

        double mTagX = cx + horiz * Math.cos(bearing);
        double mTagY = cy + horiz * Math.sin(bearing);

        if (!haveTag || Math.hypot(mTagX - tagX, mTagY - tagY) > JUMP_RESET_MM) {
            tagX = mTagX;
            tagY = mTagY;
            tagDz = dz;
            haveTag = true;
        } else {
            tagX += TAG_ALPHA * (mTagX - tagX);
            tagY += TAG_ALPHA * (mTagY - tagY);
            tagDz += HEIGHT_ALPHA * (dz - tagDz);
        }
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

        double putereStanga = forward+turn;
        double putereDreapta = forward-turn;
        stanga.setPower(putereStanga);
        dreapta.setPower(putereDreapta);

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