package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Flywheel;
import org.firstinspires.ftc.teamcode.Pozitie;

/**
 * CLUSTER tracker (4 tags, FIRST_TAG..FIRST_TAG+3 left -> right) on the motor turret (motor + pot).
 *
 * Aim point is always the CLUSTER CENTER.
 * Tag layout (manual Fig 9-15), offsets from cluster center along the row:
 *   tag FIRST_TAG+0  -165.1 mm | +1  -69.85 mm | +2  +69.85 mm | +3  +165.1 mm   (NOT evenly spaced)
 *
 * Per frame:
 *   - each tag -> field position T_i (latency-compensated camera pose + bearing + horizontal range)
 *   - 2+ tags: row direction u from the angular slope vs. offset (angles only, no range noise)
 *   - center = mean over tags of (T_i - off_i * u)   (exact, works with 1 tag once u is known)
 *
 * States:
 *   NO TAG          nothing seen yet -> turret holds.
 *   VISION (1-2)    tags visible, not locked yet -> aim at center estimated from 1-2 tags.
 *                   Tags lost before lock -> turret holds (odometry NOT trusted yet).
 *   ODOMETRY LOCK   one frame with >= MIN_TAGS_FOR_LOCK tags (default 3) -> center + row direction
 *                   measured -> locked. From now on aim from odometry every loop, with or without tags.
 *                   1-2 tag frames still refine the center (using the measured row direction),
 *                   but cannot teleport it: big jumps are only accepted from >= 3 tag frames.
 *
 * INIT: D-pad left = RED, D-pad right = BLUE. Pose loaded from Pozitie (auto). Nothing moves in INIT.
 * START: flywheel spins up.
 *
 * TWO CELLS: robot y < SWITCH_Y -> LOW cell, y > SWITCH_Y -> HIGH cell (hysteresis SWITCH_HYST).
 *   Each cell keeps its own learned center (odometry hold point). Switching swaps them.
 *   Tags of the active cell only. A DOWN cell's tags face up -> not seen -> no auto-fire at a down cell.
 *
 * CONTROLS (PS5):
 *   Square        flywheel on/off (on at start)
 *   (auto-fire)   at speed AND >= SHOOT_MIN_TAGS tags AND center locked AND turret on target
 *                 AND distance inside Flywheel.MIN_SHOT_CM .. MAX_SHOT_CM
 *   D-pad up held MANUAL OVERRIDE: fires whenever flywheel is at speed. No tag / aim check.
 *   L1 held       intake in (held off while the flywheel recovers after a shot)
 *   (auto-intake) intake also runs while stagiu feeds (INTAKE_FOLLOWS_FEED)
 *   Circle        intake + stagiu reverse
 *   L2 held       stagiu raw forward, no intake (no gating)
 *   Share         forget active cell center
 * Pan angle: robot frame, CCW +, 0 = robot front.
 */
@Config
@TeleOp(name = "teleop full", group = "turela")
public class teleopPerefect extends OpMode {

    // ================= CAMERA / MOUNT =================
    public static double MOUNT_X_MM = 0, MOUNT_Y_MM = 0;   // turret pivot from robot center
    public static double CAM_FWD_MM = 30;                   // camera ahead of pivot
    public static double CAM_PITCH_DEG = 25;   // lens axis up from horizontal (body face tilted 65 deg from floor, lens is perpendicular)

    // ================= CLUSTER =================
    // Leftmost tag of each cluster (manual: red far 30-33, red audience 34-37, blue audience 38-41, blue far 42-45)
    public static int RED_FAR_TAG = 30, RED_AUD_TAG = 34, BLUE_AUD_TAG = 38, BLUE_FAR_TAG = 42;
    public static boolean LOW_Y_IS_AUDIENCE = true; // which cell is on the y < SWITCH_Y half. CHECK on field.
    public static double SWITCH_Y_MM = 1900;        // middle of field in odometry y
    public static double SWITCH_HYST_MM = 100;      // no flip-flop when driving on the line
    // Optional preset hold points (odometry mm). Copy from telemetry "Center" after a lock. 0,0 = learn only.
    public static double LOW_CELL_X = 0, LOW_CELL_Y = 0, HIGH_CELL_X = 0, HIGH_CELL_Y = 0;
    // Tag centers along the row, from cluster center (manual Fig 9-15: 6.5 in / 2.75 in)
    public static double OFF_OUTER_MM = 165.1;
    public static double OFF_INNER_MM = 69.85;
    public static int MIN_TAGS_FOR_LOCK = 3;        // tags in one frame needed to trust odometry-lock
    public static double DIR_ALPHA = 0.2;
    public static double SIGN_CONF_SIN = 0.5;
    public static double SLOPE_DECAY = 0.9;

    // ================= TRACKING =================
    public static int PIPELINE = 2;
    public static double CENTER_ALPHA = 1;
    public static double JUMP_RESET_MM = 400;
    public static double VISION_TIMEOUT_MS = 200;   // tags "visible" if seen within this
    public static double EXTRA_LATENCY_MS = 10;
    public static double PAN_LAG_MS = 0;            // history is measured angle -> ~0
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
    public static boolean INIT_TO_CENTER = false;   // G403: TeleOp INIT must not move anything
    public static double INIT_MAX_POWER = 0.4;
    public static boolean MOTOR_REVERSED = false;   // + power raises pot V
    public static double MAX_POWER = 1;
    public static double kP = 0.006;
    public static double kD = 0.0002;
    public static double kS = 0.2;
    public static double kV = 0.0015;               // power per deg/s of target motion
    public static double TOLERANCE_DEG = 1.5;
    public static double FILTER_ALPHA = 0.4;
    public static double FF_ALPHA = 0.3;
    public static double FF_JUMP_DEG = 15;
    public static double INTAKE_POWER = 1;
    public static double STAGE_POWER=1;

    // ================= SHOOTER =================
    public static int SHOOT_MIN_TAGS = 2;           // tags in view needed for auto-fire (plus a locked center)
    public static boolean REQUIRE_AIM = true;       // auto-fire only when turret is on target
    public static double AIM_TOL_DEG = 3.0;         // turret error allowed for auto-fire
    public static boolean SPEED_FROM_DISTANCE = false; // OFF until "dist" matches a tape measure. false = FIXED_SPEED
    public static double FIXED_SPEED = 1300;          // used when SPEED_FROM_DISTANCE = false or no center yet
    public static boolean SPIN_UP_AT_START = true;
    public static boolean INTAKE_FOLLOWS_FEED = true; // intake runs with stagiu; held off while flywheel recovers
    public static double FEED_INTAKE_POWER = 1.0;     // intake power while feeding (same as AutoHelper)
    public static double RECOVER_MAX_MS = 600;        // max intake hold after a shot (if speed never comes back)
    public static boolean CAMERA_STREAM = false;      // AutoHelper: no dashboard camera stream (R704.D)

    // ================= DRIVE =================
    // Max change of drive power per second. Stops full-reverse current spikes (battery sag).
    // 0 = off (no lag). Try 8 (0 -> full in 0.125 s) only if the battery still sags hard.
    public static double DRIVE_SLEW = 0;
    // ================= HARDWARE =================
    private DcMotor stanga, dreapta, turret, intake, stagiu;
    private AnalogInput pot;
    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;
    private FtcDashboard dashboard;
    private Shooter shooter;
    private double drvL = 0, drvR = 0;            // drive power after slew limit
    private boolean flyFile = false;              // flywheel values came from the tuner file
    private boolean pSquare, pShare;
    private boolean red = false;                  // alliance, chosen in INIT
    private int activeCell = -1;                  // 0 = LOW (y < SWITCH_Y), 1 = HIGH
    private int firstTag = 38;
    private boolean poseLoaded = false;
    // saved state of each cell: haveCenter, centerLocked, haveU, uAssumed, cx, cy, ux, uy, slopeAcc
    private final boolean[][] cellFlags = new boolean[2][4];
    private final double[][] cellVals = new double[2][5];
    private int tagsNow = 0;
    private double shotDistCm = 0;
    private String fireReason = "";

    // ================= CLUSTER STATE =================
    private static final int N = 4;
    private boolean haveCenter = false, centerLocked = false, haveU = false, uAssumed = false;
    private double cx, cy;                      // cluster center (odometry frame, mm)
    private double ux = 0, uy = 1;              // row unit vector, left -> right
    private double slopeAcc = 0;

    private double lastStaleness = Double.MAX_VALUE;
    private double lastSeenMs = -1e9;
    private double lastRange, lastC;
    private int lastCnt = 0, maxCntSeen = 0;
    private String lastTags = "-", lastTx = "-", lastYaw = "-", lastNote = "";

    // ================= TURRET STATE =================
    private double v, deg, rawDeg, lastDeg;
    private boolean potOk = true;
    private double minDeg, maxDeg;
    private double desired, lastDesired, target, targetVel;
    private boolean saturated = false, aiming = false;
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
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);   // one bulk read per loop
        }
        intake = hardwareMap.dcMotor.get("intake");
        stagiu = hardwareMap.dcMotor.get("stagiu");
        stagiu.setDirection(DcMotorSimple.Direction.REVERSE);

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
        if (CAMERA_STREAM) dashboard.startCameraStream(limelight, 40);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-44.3, -6.6, DistanceUnit.MM);   // same as AutoHelper: pose must mean the same thing
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        poseLoaded = Pozitie.load();
        if (poseLoaded) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.MM, Pozitie.x, Pozitie.y, AngleUnit.RADIANS, Pozitie.h));
        } else {
            pinpoint.resetPosAndIMU();   // no auto pose: start = origin
        }
        initCells();

        stanga = hardwareMap.get(DcMotor.class, "stanga");
        dreapta = hardwareMap.get(DcMotor.class, "dreapta");
        dreapta.setDirection(DcMotorSimple.Direction.REVERSE);
        stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stagiu.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flyFile = Flywheel.load();            // KS/KV/KP + distance table from the tuner
        shooter = new Shooter(hardwareMap);   // flywheel stays off in INIT
        shooter.setTarget(FIXED_SPEED);

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

        if (gamepad1.dpad_left)  red = true;
        if (gamepad1.dpad_right) red = false;
        telemetry.addData("ALLIANCE (D-pad L=RED R=BLUE)", red ? "RED" : "BLUE");
        telemetry.addData("Flywheel values", flyFile ? "from tuner file" : "BUILT-IN (run Flywheel Tuner!)");
        telemetry.addData("Pose from auto", poseLoaded ? String.format("x %.0f y %.0f h %.1f",
                Pozitie.x, Pozitie.y, Math.toDegrees(Pozitie.h)) : "NONE -> origin");
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
        desired = lastDesired = target = deg;   // hold until something to aim at
        targetVel = 0;
        maxPowerNow = MAX_POWER;
        if (SPIN_UP_AT_START) shooter.spinUp();
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
        drive(dt);


        if (gamepad1.share && !pShare) forgetCluster();
        pShare = gamepad1.share;

        // ---------- ODOMETRY ----------
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        double rx = pose.getX(DistanceUnit.MM);
        double ry = pose.getY(DistanceUnit.MM);
        double rh = pose.getHeading(AngleUnit.RADIANS);
        double omega = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

        // ---------- CELL SELECT (by field half, with hysteresis) ----------
        int wantCell = activeCell;
        if (activeCell < 0)                              wantCell = ry > SWITCH_Y_MM ? 1 : 0;
        else if (ry > SWITCH_Y_MM + SWITCH_HYST_MM)      wantCell = 1;
        else if (ry < SWITCH_Y_MM - SWITCH_HYST_MM)      wantCell = 0;
        if (wantCell != activeCell) switchCell(wantCell);

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
        boolean visible = now - lastSeenMs < VISION_TIMEOUT_MS;
        aiming = haveCenter && (centerLocked || visible);
        if (aiming) {
            double hAhead = rh + omega * LOOKAHEAD_MS / 1000.0;
            double mx = rx + MOUNT_X_MM * Math.cos(rh) - MOUNT_Y_MM * Math.sin(rh);
            double my = ry + MOUNT_X_MM * Math.sin(rh) + MOUNT_Y_MM * Math.cos(rh);
            desired = Math.toDegrees(wrap(Math.atan2(cy - my, cx - mx) - hAhead));
        }
        // else: desired unchanged -> hold last aim

        double dDes = AngleUnit.normalizeDegrees(desired - lastDesired);
        lastDesired = desired;
        double rawVel = (!aiming || Math.abs(dDes) > FF_JUMP_DEG) ? 0 : dDes / dt;
        targetVel += FF_ALPHA * (rawVel - targetVel);

        target = pickReachable(desired);
        saturated = Math.abs(AngleUnit.normalizeDegrees(desired - target)) > 0.5;

        driveTurret(dt);
        record(now, rx, ry, rh, rawDeg);

        // ---------- SHOOTER ----------
        tagsNow = visible ? lastCnt : 0;
        if (gamepad1.square && !pSquare) shooter.toggle();
        pSquare = gamepad1.square;

        if (haveCenter) {
            double mx = rx + MOUNT_X_MM * Math.cos(rh) - MOUNT_Y_MM * Math.sin(rh);
            double my = ry + MOUNT_X_MM * Math.sin(rh) + MOUNT_Y_MM * Math.cos(rh);
            shotDistCm = Math.hypot(cx - mx, cy - my) / 10.0;
            shooter.setTarget(SPEED_FROM_DISTANCE ? Flywheel.speedForDistance(shotDistCm) : FIXED_SPEED);
        }

        boolean onTarget = !saturated && Math.abs(target - deg) <= AIM_TOL_DEG;
        boolean override = gamepad1.dpad_up;
        boolean fire;
        if (override) {
            fire = true;
            fireReason = "OVERRIDE";
        } else if (!shooter.isOn()) {
            fire = false;
            fireReason = "flywheel off";
        } else if (!centerLocked) {
            fire = false;
            fireReason = "not locked";
        } else if (SPEED_FROM_DISTANCE && shotDistCm < Flywheel.MIN_SHOT_CM) {
            fire = false;
            fireReason = String.format("TOO CLOSE (%.0f cm) - back up", shotDistCm);
        } else if (SPEED_FROM_DISTANCE && shotDistCm > Flywheel.MAX_SHOT_CM) {
            fire = false;
            fireReason = String.format("TOO FAR (%.0f cm) - move closer", shotDistCm);
        } else if (tagsNow < SHOOT_MIN_TAGS) {
            fire = false;
            fireReason = "need " + SHOOT_MIN_TAGS + " tags";
        } else if (REQUIRE_AIM && !onTarget) {
            fire = false;
            fireReason = "aiming";
        } else {
            fire = true;
            fireReason = "AUTO";
        }
        if (override && !shooter.isOn()) shooter.spinUp();   // override also spins up
        shooter.update(fire);                                 // feeds only when at speed
        // ---------- STAGIU (ONE write per loop: two writes made it pulse) ----------
        boolean l2 = gamepad1.left_trigger > 0.5;
        if (gamepad1.circle)          stagiu.setPower(-STAGE_POWER);  // reverse wins
        else if (l2)                  stagiu.setPower(STAGE_POWER);   // L2: raw stagiu, no intake
        else if (shooter.isFeeding()) stagiu.setPower(1.0);           // gated feed
        else                          stagiu.setPower(0);

        // ---------- INTAKE (after shooter: follows the feed state) ----------
        if (gamepad1.circle)                                    intake.setPower(-INTAKE_POWER);     // reverse wins
        else if (INTAKE_FOLLOWS_FEED && shooter.isFeeding())    intake.setPower(FEED_INTAKE_POWER); // push next ball
        else if (INTAKE_FOLLOWS_FEED && shooter.isRecovering()) intake.setPower(0);                 // no jam
        else if (gamepad1.left_bumper)                          intake.setPower(INTAKE_POWER);  // L1
        else                                                    intake.setPower(0);

        // ---------- TELEMETRY ----------
        double sinceSeen = (now - lastSeenMs) / 1000.0;
        String mode;
        if (!haveCenter)               mode = "NO TAG YET";
        else if (centerLocked)         mode = visible ? "LOCKED + VISION" : String.format("ODOMETRY LOCK (%.1fs)", sinceSeen);
        else if (visible)              mode = "VISION, NOT LOCKED (need " + MIN_TAGS_FOR_LOCK + " tags)";
        else                           mode = "LOST, NOT LOCKED -> HOLD";
        telemetry.addData("Alliance / cell", "%s / %s (tags %d-%d)", red ? "RED" : "BLUE",
                activeCell == 1 ? "HIGH" : "LOW", firstTag, firstTag + 3);
        telemetry.addData("TAGS VISIBLE", "%d  %s", tagsNow, tagsNow >= SHOOT_MIN_TAGS ? "(ENOUGH)" : "");
        telemetry.addData("Shooter", "%s  %.0f / %.0f t/s  %s", shooter.isOn() ? "ON" : "off",
                shooter.getVelocity(), shooter.getTarget(),
                shooter.isAtSpeed() ? "AT SPEED" : shooter.isTooFast() ? "TOO FAST"
                                                   : shooter.isRecovering() ? "RECOVERING" : "");
        telemetry.addData("Fire", "%s  shots %d  dist %.0f cm", fireReason, shooter.getShots(), shotDistCm);
        telemetry.addData("Battery", "%.2f V%s", shooter.getVolts(), shooter.getVolts() < 11.0 ? "  LOW!" : "");
        telemetry.addData("Mode", mode);
        telemetry.addData("Tags / count / max", "%s / %d / %d  (%s)", lastTags, lastCnt, maxCntSeen, lastNote);
        telemetry.addData("Per-tag tx (deg)", lastTx);
        telemetry.addData("Per-tag yaw (deg, debug)", lastYaw);
        telemetry.addData("Row dir", !haveU ? "none" : String.format("%.1f deg  %s",
                Math.toDegrees(Math.atan2(uy, ux)), uAssumed ? "ASSUMED/low-conf" : "measured"));
        telemetry.addData("View c / angle", "%.2f / %.0f deg",
                lastC, Math.toDegrees(Math.acos(clamp(lastC, -1, 1))));
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

    // ===================== VISION (cluster) =====================

    private void processFrame(LLResult r, int iPose, int iPan, double now) {
        double h = hH[iPose];
        double panR = Math.toRadians(hPan[iPan]);
        double[] off = {-OFF_OUTER_MM, -OFF_INNER_MM, OFF_INNER_MM, OFF_OUTER_MM};

        // ---- camera position at frame time ----
        double mx = hX[iPose] + MOUNT_X_MM * Math.cos(h) - MOUNT_Y_MM * Math.sin(h);
        double my = hY[iPose] + MOUNT_X_MM * Math.sin(h) + MOUNT_Y_MM * Math.cos(h);
        double camX = mx + CAM_FWD_MM * Math.cos(h + panR);
        double camY = my + CAM_FWD_MM * Math.sin(h + panR);

        boolean[] seen = new boolean[N];
        double[] bearing = new double[N], horiz = new double[N], txDeg = new double[N], yawDeg = new double[N];
        int cnt = 0;

        for (LLResultTypes.FiducialResult f : r.getFiducialResults()) {
            int i = f.getFiducialId() - firstTag;
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

        int first = -1;
        for (int i = 0; i < N; i++) if (seen[i]) { first = i; break; }
        double ref = bearing[first];

        double[] rel = new double[N];
        double sumS = 0, sumRel = 0, sumH = 0;
        StringBuilder tags = new StringBuilder(), txs = new StringBuilder(), yaws = new StringBuilder();
        for (int i = 0; i < N; i++) {
            if (!seen[i]) continue;
            rel[i] = wrap(bearing[i] - ref);
            sumS += off[i];
            sumRel += rel[i];
            sumH += horiz[i];
            tags.append(firstTag + i).append(' ');
            txs.append(String.format("%d:%.1f ", firstTag + i, txDeg[i]));
            yaws.append(String.format("%d:%.0f ", firstTag + i, yawDeg[i]));
        }
        double sm = sumS / cnt, rm = sumRel / cnt, hm = sumH / cnt;

        lastSeenMs = now;
        lastCnt = cnt;
        maxCntSeen = Math.max(maxCntSeen, cnt);
        lastTags = tags.toString();
        lastTx = txs.toString();
        lastYaw = yaws.toString();

        if (cnt >= 2) {
            // ---------- MULTI-TAG: row direction from angular slope vs. real offsets ----------
            double num = 0, den = 0, numH = 0;
            for (int i = 0; i < N; i++) {
                if (!seen[i]) continue;
                num += (off[i] - sm) * (rel[i] - rm);
                numH += (off[i] - sm) * (horiz[i] - hm);
                den += (off[i] - sm) * (off[i] - sm);
            }
            double k = num / den;                                    // rad of bearing per mm along row
            double c = clamp(-k * hm, -1, 1);                        // u . right-of-LOS
            double aC = ref + rm - sm * k;                           // LOS bearing to cluster center

            slopeAcc = SLOPE_DECAY * slopeAcc + numH / den;          // + : higher index farther
            updateRowDir(c, aC);
            lastC = c;
            lastNote = "multi";
        } else {
            // ---------- SINGLE TAG: row direction from state ----------
            if (!haveU) {                                            // first sight: face-on guess
                ux = Math.sin(bearing[first]);
                uy = -Math.cos(bearing[first]);
                haveU = true;
                uAssumed = true;
            }
            lastC = ux * Math.sin(bearing[first]) - uy * Math.cos(bearing[first]);
            lastNote = uAssumed ? "single, dir ASSUMED" : "single, dir from state";
        }

        // ---- center = mean over tags of (tag field position - offset * u) ----
        double sx = 0, sy = 0;
        for (int i = 0; i < N; i++) {
            if (!seen[i]) continue;
            double tX = camX + horiz[i] * Math.cos(bearing[i]);
            double tY = camY + horiz[i] * Math.sin(bearing[i]);
            sx += tX - off[i] * ux;
            sy += tY - off[i] * uy;
        }
        double ax = sx / cnt, ay = sy / cnt;
        lastRange = Math.hypot(ax - camX, ay - camY);

        // ---- update center, gated by lock ----
        boolean strong = cnt >= MIN_TAGS_FOR_LOCK;
        double jump = haveCenter ? Math.hypot(ax - cx, ay - cy) : 0;

        if (!haveCenter) {
            cx = ax;
            cy = ay;
            haveCenter = true;
        } else if (jump > JUMP_RESET_MM) {
            if (centerLocked && !strong) {
                lastNote += ", jump REJECTED (weak frame)";
            } else {
                cx = ax;
                cy = ay;
                uAssumed = true;                                     // cluster moved: re-learn row sign
            }
        } else {
            cx += CENTER_ALPHA * (ax - cx);
            cy += CENTER_ALPHA * (ay - cy);
        }

        if (strong) centerLocked = true;
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

    // ===================== CELLS =====================

    private int tagForCell(int cell) {
        boolean audience = (cell == 0) == LOW_Y_IS_AUDIENCE;
        if (red) return audience ? RED_AUD_TAG : RED_FAR_TAG;
        return audience ? BLUE_AUD_TAG : BLUE_FAR_TAG;
    }

    private void initCells() {
        double[][] preset = {{LOW_CELL_X, LOW_CELL_Y}, {HIGH_CELL_X, HIGH_CELL_Y}};
        for (int c = 0; c < 2; c++) {
            boolean has = preset[c][0] != 0 || preset[c][1] != 0;
            cellFlags[c] = new boolean[]{has, has, false, false};   // preset = locked hold point
            cellVals[c] = new double[]{preset[c][0], preset[c][1], 0, 1, 0};
        }
        activeCell = -1;
    }

    private void saveCell(int c) {
        cellFlags[c] = new boolean[]{haveCenter, centerLocked, haveU, uAssumed};
        cellVals[c] = new double[]{cx, cy, ux, uy, slopeAcc};
    }

    private void loadCell(int c) {
        haveCenter = cellFlags[c][0]; centerLocked = cellFlags[c][1];
        haveU = cellFlags[c][2];      uAssumed = cellFlags[c][3];
        cx = cellVals[c][0]; cy = cellVals[c][1];
        ux = cellVals[c][2]; uy = cellVals[c][3]; slopeAcc = cellVals[c][4];
    }

    private void switchCell(int c) {
        if (activeCell >= 0) saveCell(activeCell);
        activeCell = c;
        firstTag = tagForCell(c);
        loadCell(c);
        maxCntSeen = 0;
        lastSeenMs = -1e9;          // old cell's tags no longer count as visible
        lastNote = "switched cell";
    }

    private void forgetCluster() {
        haveCenter = centerLocked = haveU = uAssumed = false;
        ux = 0; uy = 1;
        slopeAcc = 0;
        maxCntSeen = 0;
        lastNote = "forgotten";
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

    private void drive(double dt) {
        double fwd  =  gamepad1.left_stick_y;   // inverted: stick up drives the other way (chassis front)
        double turn =  -gamepad1.right_stick_x;
        double max = Math.max(1, Math.abs(fwd) + Math.abs(turn));
        drvL = slew(drvL, (fwd + turn) / max, dt);
        drvR = slew(drvR, (fwd - turn) / max, dt);
        stanga.setPower(drvL);
        dreapta.setPower(drvR);
    }

    /** Move toward want by at most DRIVE_SLEW per second. */
    private static double slew(double now, double want, double dt) {
        if (DRIVE_SLEW <= 0) return want;
        double step = DRIVE_SLEW * dt;
        return now + clamp(want - now, -step, step);
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
        if (intake != null) intake.setPower(0);
        if (stagiu != null) stagiu.setPower(0);
        if (stanga != null) stanga.setPower(0);
        if (dreapta != null) dreapta.setPower(0);
        if (shooter != null) shooter.stop();
        if (dashboard != null) dashboard.stopCameraStream();
        if (limelight != null) limelight.stop();
    }

    // ===================== SHOOTER (flywheel via Flywheel class + shot counting) =====================
    // Speed control + battery compensation + at-speed gating live in Flywheel (same as tuner + auto).
    // Feeder stagiu is written once per loop in loop(), not here.
    static class Shooter {
        private final Flywheel fw;
        private boolean wasFeeding = false, feeding = false, recovering = false;
        private final ElapsedTime recoverTimer = new ElapsedTime();
        private int shots = 0;

        public Shooter(HardwareMap hw) { fw = new Flywheel(hw); }   // nothing moves: safe for INIT

        public void setTarget(double ticksPerSec) { fw.setTarget(ticksPerSec); }
        public double getTarget() { return fw.getTarget(); }
        public void spinUp() { fw.setOn(true); }
        public void spinDown() { fw.setOn(false); }
        public boolean isOn() { return fw.isOn(); }
        public void toggle() { fw.setOn(!fw.isOn()); }
        public boolean isAtSpeed() { return fw.isAtSpeed(); }
        public boolean isTooFast() {
            return fw.isOn() && fw.getTarget() > 0 && fw.getVelocity() > fw.getTarget() * (1 + Flywheel.OVER_BAND);
        }
        public int getShots() { return shots; }
        public boolean isFeeding() { return feeding; }
        public boolean isRecovering() { return recovering; }
        public double getVelocity() { return fw.getVelocity(); }
        public double getVolts() { return fw.getVolts(); }

        /** Call every loop. fire = driver/auto wants to shoot now. */
        public void update(boolean fire) {
            fw.update();
            boolean dipped = fw.getVelocity() < fw.getTarget() * (1 - Flywheel.SHOT_DIP);
            feeding = fire && fw.isAtSpeed();
            if (wasFeeding && !feeding && fire && dipped) {   // feeder cut by a dip = one ball fired
                shots++;
                recovering = true;                           // hold intake until speed is back
                recoverTimer.reset();
            }
            if (fw.isAtSpeed() || recoverTimer.milliseconds() > RECOVER_MAX_MS) recovering = false;
            wasFeeding = feeding;
        }

        public void stop() { fw.stop(); }
    }
}
