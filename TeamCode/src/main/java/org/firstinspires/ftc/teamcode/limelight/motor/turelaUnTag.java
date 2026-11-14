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
 * SINGLE-TAG tracker (Limelight 3A) on the motor turret (motor + pot, no encoder).
 *
 * Tag seen  -> tag position stored in odometry frame (latency-compensated).
 * Every loop -> aim at stored tag position from odometry (works between frames / tag lost).
 * Init      -> turret drives to center (V_CENTER) and holds.
 *
 * Pan angle: robot frame, CCW +, 0 = robot front.
 */
@Config
@TeleOp(name = "turela n tag", group = "turela")
public class turelaUnTag extends OpMode {

    // ================= TAG =================
    public static double CAM_FWD_MM = 30;
    public static double CAM_PITCH_DEG = 0;
    public static double EXTRA_LATENCY_MS = 10;
    public static double FF_ALPHA = 0.3;
    public static double FF_JUMP_DEG = 15;
    public static double FILTER_ALPHA = 0.4;
    public static double INIT_MAX_POWER = 0.8;
    public static boolean INIT_TO_CENTER = true;
    public static double JUMP_RESET_MM = 400;
    public static boolean LATERAL_IS_LEFT = false;
    public static double LOOKAHEAD_MS = 30;
    public static double MAX_DIST_MM = 6000;
    public static double MAX_POWER = 1;
    public static double MIN_DIST_MM = 200;
    public static boolean MOTOR_REVERSED = false;
    public static double MOUNT_X_MM = 0;
    public static double MOUNT_Y_MM = 0;
    public static double PAN_LAG_MS = 0;
    public static int PIPELINE = 2;
    public static double TAG_ALPHA = 0.3;
    public static int TAG_ID = 38;
    public static double TOLERANCE_DEG = 1.5;
    public static double V_BACK = 1.5;
    public static double V_CENTER = 1.03;
    public static double V_FRONT = 0.663;
    public static double V_LIMIT_MAX = 1.75;
    public static double V_LIMIT_MIN = 0.5;
    public static double kD = 0.0002;
    public static double kP = 0.006;
    public static double kS = 0.15;
    public static double kV = 0.0015;
    // ================= HARDWARE =================
    private DcMotor stanga, dreapta, turret;
    private AnalogInput pot;
    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;
    private FtcDashboard dashboard;

    // ================= TAG STATE =================
    private boolean haveTag = false;
    private double tagX, tagY;                     // odometry frame, mm
    private double lastStaleness = Double.MAX_VALUE;
    private double lastSeenMs = -1e9;
    private double lastTx = 0, lastRange = 0;

    // ================= TURRET STATE =================
    private double v, deg, rawDeg, lastDeg;
    private boolean potOk = true;
    private double minDeg, maxDeg;
    private double holdDeg, desired, lastDesired, target, targetVel;
    private boolean saturated = false;
    private double motorPower = 0, maxPowerNow = 1;

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
        turret.setDirection(MOTOR_REVERSED ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);

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

        computeLimits();
        v = pot.getVoltage();
        deg = rawDeg = lastDeg = turretDeg(v);
        runtime.reset();
        lastLoopMs = 0;
    }

    @Override
    public void init_loop() {
        double now = runtime.milliseconds();
        double dt = (now - lastLoopMs) / 1000.0;
        lastLoopMs = now;
        if (dt <= 0) return;

        computeLimits();
        readTurret();

        if (INIT_TO_CENTER) {
            target = clamp(turretDeg(V_CENTER), minDeg, maxDeg);
            targetVel = 0;
            saturated = false;
            maxPowerNow = INIT_MAX_POWER;
            driveTurret(dt);
        } else {
            turret.setPower(0);
        }

        telemetry.addLine(INIT_TO_CENTER ? "INIT: moving turret to center" : "INIT: turret idle");
        telemetry.addData("Pot V / target V", "%.3f / %.3f", v, V_CENTER);
        telemetry.addData("Turret deg / target", "%.1f / %.1f", deg, target);
        telemetry.addData("Reach", "%.0f .. %.0f", minDeg, maxDeg);
        telemetry.addData("Power", "%.2f", motorPower);
        telemetry.update();
    }

    @Override
    public void start() {
        readTurret();
        holdDeg = desired = lastDesired = target = deg;   // hold until tag seen
        targetVel = 0;
        maxPowerNow = MAX_POWER;
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

        maxPowerNow = MAX_POWER;
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

        // ---------- AIM ----------
        if (haveTag) {
            double hAhead = rh + omega * LOOKAHEAD_MS / 1000.0;
            double mx = rx + MOUNT_X_MM * Math.cos(rh) - MOUNT_Y_MM * Math.sin(rh);
            double my = ry + MOUNT_X_MM * Math.sin(rh) + MOUNT_Y_MM * Math.cos(rh);
            desired = Math.toDegrees(wrap(Math.atan2(tagY - my, tagX - mx) - hAhead));
        } else {
            desired = holdDeg;
        }

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
        telemetry.addData("Mode", !haveTag ? "NO TAG YET"
                : sinceSeen < 0.2 ? "VISION LOCK" : String.format("ODOMETRY ONLY (%.1fs)", sinceSeen));
        telemetry.addData("Tag", "%d  tx %.1f  range %.0f mm", TAG_ID, lastTx, lastRange);
        telemetry.addData("Tag pos (mm)", "x %.0f  y %.0f", tagX, tagY);
        telemetry.addData("Pan desired / target / actual", "%.1f / %.1f / %.1f", desired, target, deg);
        telemetry.addData("SATURATED", saturated ? "PAN (blind zone)" : "");
        telemetry.addData("Target vel / power", "%.0f deg/s / %.2f", targetVel, motorPower);
        telemetry.addData("Pot V / ok", "%.3f / %b", v, potOk);
        telemetry.addData("Robot", "x %.0f  y %.0f  h %.1f", rx, ry, Math.toDegrees(rh));
        telemetry.addData("Staleness", "%.0f ms", staleness);
        telemetry.addData("Loop", "%.1f ms", loopMs);
        telemetry.update();
    }

    // ===================== VISION (single tag) =====================

    private void processFrame(LLResult r, int iPose, int iPan, double now) {
        for (LLResultTypes.FiducialResult f : r.getFiducialResults()) {
            if (f.getFiducialId() != TAG_ID) continue;

            Position p = f.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.MM);
            double range = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            if (range <= MIN_DIST_MM || range >= MAX_DIST_MM) return;

            double tx = f.getTargetXDegrees(), ty = f.getTargetYDegrees();
            double h = hH[iPose];
            double panR = Math.toRadians(hPan[iPan]);
            double bearing = h + panR - Math.toRadians(tx);                 // tx + to the right
            double horiz = range * Math.cos(Math.toRadians(CAM_PITCH_DEG + ty));

            // camera position at frame time -> tag in odometry frame
            double mx = hX[iPose] + MOUNT_X_MM * Math.cos(h) - MOUNT_Y_MM * Math.sin(h);
            double my = hY[iPose] + MOUNT_X_MM * Math.sin(h) + MOUNT_Y_MM * Math.cos(h);
            double camX = mx + CAM_FWD_MM * Math.cos(h + panR);
            double camY = my + CAM_FWD_MM * Math.sin(h + panR);
            double ax = camX + horiz * Math.cos(bearing);
            double ay = camY + horiz * Math.sin(bearing);

            if (!haveTag || Math.hypot(ax - tagX, ay - tagY) > JUMP_RESET_MM) {
                tagX = ax;
                tagY = ay;
                haveTag = true;
            } else {
                tagX += TAG_ALPHA * (ax - tagX);
                tagY += TAG_ALPHA * (ay - tagY);
            }

            lastSeenMs = now;
            lastTx = tx;
            lastRange = horiz;
            return;
        }
    }

    // ===================== TURRET =====================

    private void readTurret() {
        double raw = pot.getVoltage();
        potOk = raw > 0.03 && raw < 3.25;
        if (!potOk) return;
        v += FILTER_ALPHA * (raw - v);
        rawDeg = turretDeg(raw);     // unfiltered -> vision history
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
        power = clamp(power, -maxPowerNow, maxPowerNow);

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