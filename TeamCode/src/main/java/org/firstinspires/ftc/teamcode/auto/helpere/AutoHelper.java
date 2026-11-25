package org.firstinspires.ftc.teamcode.auto.helpere;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Flywheel;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/**
 * Drive (go-to-point + heading) + turret cluster tracking (from teleop) + flywheel with velocity gating (from ShootSimple).
 *
 * OpMode usage (iterative):
 *   init():       drive = AutoHelper.fromHardwareMap(...)
 *                 drive.setup(tag, vInit, feedNeedsTags, flyTarget)   (optional, see setup())
 *   init_loop():  drive.turretInitLoop(telemetry)     -> turret goes to V_INIT (R103.B: allowed in AUTO INIT)
 *   start():      drive.startTurret()                  -> seeds hive center from current pan + HIVE_DIST_MM
 *   loop():       drive.update(telemetry) FIRST, then lap()/shootFor()/...
 *   stop():       drive.stopAll()
 */
@Config
public class AutoHelper {

    // ===================== COR =====================
    // Pozitia COR fata de punctul urmarit, in cadrul robotului
//    public static double COR_FORWARD_MM = 0;
//    public static double COR_LEFT_MM = 0;
//
//    // ===================== GO-TO-POINT =====================
//    public static double DISTANCE_TOLERANCE_MM = 15;
//    public static double MIN_MOVE_MM = 80;
//    public static double HEADING_LOCK_DEG = 3;
//    public static double KP_DRIVE = 0.0022;
//    public static double MAX_POWER = 0.5;
//    public static double MIN_DRIVE_POWER = 0.15;
//    public static double KP_TURN = 0.02;
//    public static double MAX_TURN = 0.3;
//    public static double LOOKAHEAD_MM = 200;
//    public static double MAX_CORRECTION_DEG = 20;
//    public static double FADE_DISTANCE_MM = 150;
//
//    // ===================== HEADING PE LOC =====================
//    public static double HEADING_KP = 0.55;
//    public static double HEADING_TANH_SCALE = 2.0;
//    public static double HEADING_KS = 0.2;
//    public static double HEADING_KD = 0.002;
//    public static double HEADING_DEADBAND = 0.4;
//    public static double HEADING_SETTLE_RATE = 15;
//    public static double HEADING_MAX_POWER = 1.0;
//    public static double NOMINAL_VOLTAGE = 12.5;
//
//    // ===================== FLYWHEEL (ShootSimple) =====================
//    public static double FLY_KP = 24, FLY_KI = 2.4, FLY_KD = 0, FLY_KF = 0;
//    public static double FLY_TARGET = 1200;        // ticks/s
//    public static double READY_BAND = 0.01;        // feed only above 99% of target
//    public static double SHOT_DIP = 0.025;         // below 97.5% = ball fired -> stop stagiu
//    public static double FEED_POWER = 1.0;
//    public static double AIM_TOL_DEG = 3;          // turret must be within this of target to feed
//    public static double SPINUP_TIMEOUT_MS = 2500; // feed window starts anyway after this
//
//    // ===================== TURRET: CAMERA / MOUNT =====================
//    public static double MOUNT_X_MM = 0, MOUNT_Y_MM = 0;
//    public static double CAM_FWD_MM = 30;
//    public static double CAM_PITCH_DEG = 25;
//
//    // ===================== TURRET: CLUSTER =====================
//    public static int FIRST_TAG = 34;              // red audience CELL (up at match start): 34 35 36 37
//    public static double OFF_OUTER_MM = 165.1;
//    public static double OFF_INNER_MM = 69.85;
//    public static int MIN_TAGS_FOR_LOCK = 3;
//    public static double DIR_ALPHA = 0.2;
//    public static double SIGN_CONF_SIN = 0.5;
//    public static double SLOPE_DECAY = 0.9;
//    public static double HIVE_DIST_MM = 1500;      // MEASURE: horizontal turret pivot -> cluster center at start pose
//
//    // ===================== TURRET: TRACKING =====================
//    public static int PIPELINE = 2;
//    public static double CENTER_ALPHA = 1;
//    public static double JUMP_RESET_MM = 400;
//    public static double VISION_TIMEOUT_MS = 200;
//    public static double EXTRA_LATENCY_MS = 10;
//    public static double PAN_LAG_MS = 0;
//    public static double TURRET_LOOKAHEAD_MS = 30;
//    public static double MIN_DIST_MM = 200, MAX_DIST_MM = 6000;
//
//    // ===================== TURRET: POT =====================
//    public static double V_FRONT  = 0.663;
//    public static double V_CENTER = 1.030;
//    public static double V_BACK   = 1.500;
//    public static double V_LIMIT_MIN = 0.5;
//    public static double V_LIMIT_MAX = 1.75;
//    public static boolean LATERAL_IS_LEFT = false;
//
//    // ===================== TURRET: CONTROL =====================
//    public static double INIT_MAX_POWER = 0.4;
//    public static boolean MOTOR_REVERSED = false;
//    public static double TURRET_MAX_POWER = 1;
//    public static double kP = 0.006;
//    public static double kD = 0.0002;
//    public static double kS = 0.2;
//    public static double kV = 0.0015;
//    public static double TOLERANCE_DEG = 1.5;
//    public static double FILTER_ALPHA = 0.4;
//    public static double FF_ALPHA = 0.3;
//    public static double FF_JUMP_DEG = 15;

    public static double AIM_TOL_DEG = 3;
    public static double CAM_FWD_MM = 30;
    public static double CAM_PITCH_DEG = 25;
    public static double CENTER_ALPHA = 1;
    public static double COR_FORWARD_MM = 0;
    public static double COR_LEFT_MM = 0;
    public static double DIR_ALPHA = 0.2;
    public static double DISTANCE_TOLERANCE_MM = 15;
    public static double EXTRA_LATENCY_MS = 10;
    public static double FADE_DISTANCE_MM = 150;
    public static double FEED_POWER = 1;
    public static double FF_ALPHA = 0.3;
    public static double FF_JUMP_DEG = 15;
    public static double FILTER_ALPHA = 0.4;
    public static int FIRST_TAG = 34;
    // Flywheel speed comes from Flywheel's distance table (tuner) using the hive distance.
    // FLY_TARGET is only used if there is no hive center yet.
    public static boolean FLY_FROM_DISTANCE = false;   // OFF until the distance is verified
    public static double FLY_TARGET = 1300;
    public static double HEADING_DEADBAND = 0.4;
    public static double HEADING_KD = 0.002;
    public static double HEADING_KP = 0.55;
    public static double HEADING_KS = 0.35;
    public static double HEADING_LOCK_DEG = 5;
    public static double HEADING_MAX_POWER = 1;
    public static double HEADING_SETTLE_RATE = 15;
    public static double HEADING_TANH_SCALE = 2;
    public static double HIVE_DIST_MM = 1500;
    public static double INIT_MAX_POWER = 1;
    public static double JUMP_RESET_MM = 400;
    public static double KP_DRIVE = 0.0022;
    public static double KP_TURN = 0.02;
    public static boolean LATERAL_IS_LEFT = false;
    public static double LOOKAHEAD_MM = 200;
    public static double MAX_CORRECTION_DEG = 20;
    public static double MAX_DIST_MM = 6000;
    public static double MAX_POWER = 0.4;
    public static double MAX_TURN = 1;
    public static double MIN_DIST_MM = 200;
    public static double MIN_DRIVE_POWER = 0.3;
    public static double MIN_MOVE_MM = 80;
    public static int MIN_TAGS_FOR_LOCK = 3;
    public static boolean MOTOR_REVERSED = false;
    public static double MOUNT_X_MM = 0;
    public static double MOUNT_Y_MM = 0;
    public static double NOMINAL_VOLTAGE = 12.5;
    public static double OFF_INNER_MM = 69.85;
    public static double OFF_OUTER_MM = 165.1;
    public static double PAN_LAG_MS = 0;
    public static int PIPELINE = 2;
    public static double SIGN_CONF_SIN = 0.5;
    public static double SLOPE_DECAY = 0.9;
    public static double SPINUP_TIMEOUT_MS = 2500;
    public static double TOLERANCE_DEG = 3;
    public static double TURRET_LOOKAHEAD_MS = 30;
    public static double TURRET_MAX_POWER = 1;
    public static double VISION_TIMEOUT_MS = 200;
    public static double V_BACK = 1.5;
    public static double V_CENTER = 1.03;
    public static double V_FRONT = 0.663;
    public static double V_LIMIT_MAX = 1.75;
    public static double V_LIMIT_MIN = 0.5;
    public static double V_INIT = 1.03;           // turret pot V in INIT: 1.03 = side (V_CENTER), 0.663 = forward
    public static boolean FEED_NEEDS_TAGS = false; // true = feed only while tags are seen live (no odometry-only shots)
    public static double kD = 0.0002;
    public static double kP = 0.006;
    public static double kS = 0.2;
    public static double kV = 0.0015;

    // ===================== HARDWARE =====================
    public static DcMotor stanga;
    public static DcMotor dreapta;
    private final GoBildaPinpointDriver pinpoint;
    private final VoltageSensor voltageSensor;
    private final Flywheel fly;
    private final DcMotor stagiu, turret, intake;
    private final AnalogInput pot;
    private final Limelight3A limelight;

    // ===================== DRIVE STATE =====================
    private Pose2D pose;
    private boolean poseFresh = false;
    private boolean doneXY = false;
    private boolean doneHeading = false;
    private double targetX = Double.NaN;
    private double targetY = Double.NaN;
    private boolean lineReady = false;
    private boolean facing = false;
    private double lineStartX, lineStartY, lineHeading;

    // ===================== SHOOTER STATE =====================
    private double flyTarget = 0, flyVel = 0, shotDistCm = 0;
    private boolean atSpeed = false, onTarget = false;
    private boolean shooting = false, windowStarted = false;
    private final ElapsedTime shootTimer = new ElapsedTime();

    // ===================== CLUSTER STATE =====================
    private static final int N = 4;
    private boolean haveCenter = false, centerLocked = false, seeded = false, haveU = false, uAssumed = false;
    private boolean everSeen = false;   // true after the first real tag frame (the seed does not count)
    private double cx, cy;
    private double ux = 0, uy = 1;
    private double slopeAcc = 0;
    private double lastStaleness = Double.MAX_VALUE;
    private double lastSeenMs = -1e9;
    private int lastCnt = 0;
    private String lastNote = "";

    // ===================== TURRET STATE =====================
    private double v, deg, rawDeg, lastDeg;
    private boolean potOk = true;
    private double minDeg, maxDeg;
    private double desired, lastDesired, target, targetVel;
    private boolean saturated = false, aiming = false;
    private double motorPower = 0, maxPowerNow = 1;

    private final ElapsedTime clock = new ElapsedTime();   // never reset: history timestamps
    private double lastTurretMs = 0;

    private static final int HIST = 256;
    private final double[] hT = new double[HIST], hX = new double[HIST], hY = new double[HIST],
            hH = new double[HIST], hPan = new double[HIST];
    private int hIdx = 0, hCount = 0;

    // =========================================================

    public AutoHelper(DcMotor stanga, DcMotor dreapta,
                      GoBildaPinpointDriver pinpoint, HardwareMap hardwareMap) {
        AutoHelper.stanga = stanga;
        AutoHelper.dreapta = dreapta;
        this.pinpoint = pinpoint;
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // flywheel
        Flywheel.load();                       // tuner values if the file exists, else built-in
        fly = new Flywheel(hardwareMap);
        intake = hardwareMap.dcMotor.get("intake");
        stagiu = hardwareMap.dcMotor.get("stagiu");
        stagiu.setDirection(DcMotorSimple.Direction.REVERSE);
        stagiu.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // turret
        turret = hardwareMap.get(DcMotor.class, "turret");
        pot = hardwareMap.get(AnalogInput.class, "potTurela");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(MOTOR_REVERSED ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);

        // limelight (no dashboard camera stream: R704.D)
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();

        computeLimits();
        v = pot.getVoltage();
        deg = rawDeg = lastDeg = turretDeg(v);
        clock.reset();
    }

    /** Same as below, and sets FIRST_TAG (leftmost tag of the CELL to aim at: 30, 34, 38 or 42). */
    public static AutoHelper fromHardwareMap(HardwareMap hardwareMap,
                                             String leftName, String rightName, String pinpointName,
                                             double podOffsetX_mm, double podOffsetY_mm, int tag) {
        FIRST_TAG = tag;
        return fromHardwareMap(hardwareMap, leftName, rightName, pinpointName, podOffsetX_mm, podOffsetY_mm);
    }

    public static AutoHelper fromHardwareMap(HardwareMap hardwareMap,
                                             String leftName, String rightName, String pinpointName,
                                             double podOffsetX_mm, double podOffsetY_mm) {
        DcMotor stanga = hardwareMap.get(DcMotor.class, leftName);
        DcMotor dreapta = hardwareMap.get(DcMotor.class, rightName);

        stanga.setDirection(DcMotorSimple.Direction.REVERSE);
        dreapta.setDirection(DcMotorSimple.Direction.FORWARD);
        stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, pinpointName);
        pinpoint.setOffsets(podOffsetX_mm, podOffsetY_mm, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        return new AutoHelper(stanga, dreapta, pinpoint, hardwareMap);
    }

    /**
     * Set the per-auto values in one call (in init, after fromHardwareMap).
     *   firstTag       leftmost tag of the CELL: 30 red far, 34 red audience, 38 blue audience, 42 blue far
     *   vInit          turret pot V in INIT: 1.03 = side, 0.663 = forward
     *   feedNeedsTags  true = feed only while tags are seen live ("Tilted" behaviour)
     *   flyTarget      flywheel ticks/s (used when FLY_FROM_DISTANCE = false)
     * Example (AutoTrasTiltJos red):  drive.setup(30, 0.663, true, 1200);
     */
    public AutoHelper setup(int firstTag, double vInit, boolean feedNeedsTags, double flyTarget) {
        FIRST_TAG = firstTag;
        V_INIT = vInit;
        FEED_NEEDS_TAGS = feedNeedsTags;
        FLY_TARGET = flyTarget;
        return this;
    }

    /** True after the first real tag frame since startTurret(). The seed does not count. */
    public boolean tagsSeen() { return everSeen; }

    /** True if a tag frame arrived in the last VISION_TIMEOUT_MS. */
    public boolean tagsVisible() { return clock.milliseconds() - lastSeenMs < VISION_TIMEOUT_MS; }

    // =========================================================
    //                    MAIN UPDATE (call first in loop)
    // =========================================================

    /** One pinpoint read per loop + turret tracking. Call at the top of every loop(). */
    public void update(Telemetry t) {
        refreshPose();
        poseFresh = true;
        updateTurret(t);
        fly.update();                          // flywheel control every loop (pre-spin works too)
    }

    private void ensurePose() {
        if (!poseFresh || pose == null) refreshPose();   // OpModes that never call update() still work
    }

    // =========================================================
    //                    GO-TO-POINT (COR)
    // =========================================================

    public void updateXY(double targetX_mm, double targetY_mm,
                         double finalHeadingDeg, Telemetry t) {
        ensurePose();
        if (targetX_mm != targetX || targetY_mm != targetY) {
            lineReady = false;
        }
        targetX = targetX_mm;
        targetY = targetY_mm;

        double heading = pose.getHeading(DEGREES);
        double[] offsetNow = rotate(COR_FORWARD_MM, COR_LEFT_MM, heading);
        double[] offsetFinal = rotate(COR_FORWARD_MM, COR_LEFT_MM, finalHeadingDeg);

        double corX = pose.getX(DistanceUnit.MM) + offsetNow[0];
        double corY = pose.getY(DistanceUnit.MM) + offsetNow[1];

        double goalX = targetX_mm + offsetFinal[0];
        double goalY = targetY_mm + offsetFinal[1];

        if (!lineReady) {
            startLine(corX, corY, goalX, goalY, heading, finalHeadingDeg);
        }

        double cos = Math.cos(Math.toRadians(lineHeading));
        double sin = Math.sin(Math.toRadians(lineHeading));
        double along = (goalX - corX) * cos + (goalY - corY) * sin;
        double cross = (corX - lineStartX) * -sin + (corY - lineStartY) * cos;

        t.addData("along", along);
        t.addData("cross", cross);

        if (Math.abs(along) <= DISTANCE_TOLERANCE_MM) {
            stop();
            doneXY = true;
            return;
        }

        doneXY = false;

        if (facing) {
            double faceError = normalizeAngle(lineHeading - heading);
            t.addData("heading error", faceError);
            if (Math.abs(faceError) > HEADING_LOCK_DEG) {
                setTurn(headingPower(faceError));
                return;
            }
            facing = false;
        }

        double direction = Math.signum(along);
        double correction = Math.toDegrees(Math.atan2(-cross, LOOKAHEAD_MM));
        double fade = Math.min(1.0, Math.abs(along) / FADE_DISTANCE_MM);
        double desiredHeading = lineHeading
                + Range.clip(correction, -MAX_CORRECTION_DEG, MAX_CORRECTION_DEG) * direction * fade;
        double headingError = normalizeAngle(desiredHeading - heading);

        t.addData("heading error", headingError);

        double turn = Range.clip(KP_TURN * headingError, -MAX_TURN, MAX_TURN);
        double drive = direction * Range.clip(Math.abs(KP_DRIVE * along), MIN_DRIVE_POWER, MAX_POWER);

        setDriveTurn(drive, turn);
    }

    private void startLine(double corX, double corY, double goalX, double goalY,
                           double heading, double finalHeading) {
        lineStartX = corX;
        lineStartY = corY;
        lineReady = true;

        if (Math.hypot(goalX - corX, goalY - corY) <= MIN_MOVE_MM) {
            lineHeading = heading;
            facing = false;
            return;
        }

        double bearing = Math.toDegrees(Math.atan2(goalY - corY, goalX - corX));
        double reverse = normalizeAngle(bearing + 180.0);

        lineHeading = totalTurn(heading, reverse, finalHeading) < totalTurn(heading, bearing, finalHeading)
                ? reverse
                : bearing;

        facing = true;
    }

    // =========================================================
    //               HEADING DOAR PE LOC
    // =========================================================

    public void updateHeading(double targetHeadingDeg, Telemetry t) {
        ensurePose();

        double current = pose.getHeading(DEGREES);
        double error = normalizeAngle(targetHeadingDeg - current);
        double rate = headingRate();

        t.addData("target heading", targetHeadingDeg);
        t.addData("current heading", current);
        t.addData("heading error", error);
        t.addData("heading rate", rate);
        if (!Double.isNaN(targetX)) {
            t.addData("point error", Math.hypot(
                    targetX - pose.getX(DistanceUnit.MM),
                    targetY - pose.getY(DistanceUnit.MM)));
        }

        if (Math.abs(error) <= HEADING_DEADBAND) {
            stop();
            doneHeading = Math.abs(rate) <= HEADING_SETTLE_RATE;
            return;
        }

        setTurn(headingPower(error));
        doneHeading = false;
    }

    public boolean lap(double targetHeadingDeg, double targetX_mm,
                       double targetY_mm, Telemetry t) {
        if (!doneXY) {
            updateXY(targetX_mm, targetY_mm, targetHeadingDeg, t);
        } else if (!doneHeading) {
            updateHeading(targetHeadingDeg, t);
        }
        return doneXY && doneHeading;
    }

    public void resetMove() {
        doneXY = false;
        doneHeading = false;
        lineReady = false;
        facing = false;
    }

    // =========================================================
    //                    SHOOTER (velocity gating)
    // =========================================================

    public void setFlywheel(double target) {
        flyTarget = target;
        fly.setTarget(target);
        fly.setOn(target > 0);
    }

    /** ShootSimple gating + turret on target. Returns true when ready to feed. */
    public boolean updateShooter(boolean feed) {
        fly.update();                          // battery-compensated speed control + at-speed gating
        flyVel = fly.getVelocity();
        atSpeed = fly.isAtSpeed();
        onTarget = aiming && !saturated && Math.abs(target - deg) <= AIM_TOL_DEG
                && (!FEED_NEEDS_TAGS || tagsVisible());
        boolean ready = atSpeed && onTarget;
        double p = (feed && ready) ? 1 : 0;
        intake.setPower(p);
        stagiu.setPower(p * FEED_POWER);
        return ready;
    }

    /**
     * Call every loop. Spins flywheel to FLY_TARGET, waits for speed + aim, then feeds (gated) for feedMs.
     * Returns true when done; flywheel and stagiu are then off.
     */
    public boolean shootFor(double feedMs, Telemetry t) {
        if (!shooting) {
            shooting = true;
            windowStarted = false;
            shootTimer.reset();
        }
        setFlywheel(shotSpeed());
        boolean ready = updateShooter(windowStarted);

        if (!windowStarted && (ready || shootTimer.milliseconds() >= SPINUP_TIMEOUT_MS)) {
            windowStarted = true;
            shootTimer.reset();
        }

        t.addData("fly vel / target", "%.0f / %.0f  (dist %.0f cm%s)", flyVel, flyTarget, shotDistCm,
                Flywheel.inRange(shotDistCm) ? "" : " OUT OF RANGE");
        t.addData("battery", "%.2f V", fly.getVolts());
        t.addData("tags visible", "%b%s", tagsVisible(), FEED_NEEDS_TAGS ? " (needed to feed)" : "");
        t.addData("at speed / on target / window", "%b / %b / %b", atSpeed, onTarget, windowStarted);
        t.addData("shoot ms", "%.0f", shootTimer.milliseconds());

        if (windowStarted && shootTimer.milliseconds() >= feedMs) {
            stopShooter();
            return true;
        }
        return false;
    }

    /** Speed from the distance table (turret pivot -> hive center), else FLY_TARGET. */
    private double shotSpeed() {
        if (!FLY_FROM_DISTANCE || !haveCenter || pose == null) return FLY_TARGET;
        double rx = pose.getX(DistanceUnit.MM), ry = pose.getY(DistanceUnit.MM);
        double rh = pose.getHeading(AngleUnit.RADIANS);
        double mx = rx + MOUNT_X_MM * Math.cos(rh) - MOUNT_Y_MM * Math.sin(rh);
        double my = ry + MOUNT_X_MM * Math.sin(rh) + MOUNT_Y_MM * Math.cos(rh);
        shotDistCm = Math.hypot(cx - mx, cy - my) / 10.0;
        return Flywheel.speedForDistance(shotDistCm);
    }

    public void stopShooter() {
        stagiu.setPower(0);
        intake.setPower(0);
        setFlywheel(0);
        fly.stop();
        shooting = false;
        windowStarted = false;
        atSpeed = false;
    }

    // =========================================================
    //                    TURRET (from teleop)
    // =========================================================

    /** init_loop(): drive turret to V_INIT at INIT_MAX_POWER. */
    public void turretInitLoop(Telemetry t) {
        double now = clock.milliseconds();
        double dt = (now - lastTurretMs) / 1000.0;
        lastTurretMs = now;
        if (dt <= 0) return;

        computeLimits();
        readTurret();
        target = clamp(turretDeg(V_INIT), minDeg, maxDeg);
        targetVel = 0;
        saturated = false;
        maxPowerNow = INIT_MAX_POWER;
        driveTurret(dt);

        t.addLine("INIT: turret -> V_INIT");
        t.addData("Pot V / target V", "%.3f / %.3f", v, V_INIT);
        t.addData("Turret deg / target", "%.1f / %.1f", deg, target);
    }

    /** start(): seed hive center along the current turret direction at HIVE_DIST_MM. Vision corrects it. */
    public void startTurret() {
        refreshPose();
        readTurret();
        desired = lastDesired = target = deg;
        targetVel = 0;
        maxPowerNow = TURRET_MAX_POWER;
        lastTurretMs = clock.milliseconds();

        double rx = pose.getX(DistanceUnit.MM), ry = pose.getY(DistanceUnit.MM);
        double rh = pose.getHeading(AngleUnit.RADIANS);
        double mx = rx + MOUNT_X_MM * Math.cos(rh) - MOUNT_Y_MM * Math.sin(rh);
        double my = ry + MOUNT_X_MM * Math.sin(rh) + MOUNT_Y_MM * Math.cos(rh);
        double a = rh + Math.toRadians(deg);
        cx = mx + HIVE_DIST_MM * Math.cos(a);
        cy = my + HIVE_DIST_MM * Math.sin(a);
        haveCenter = true;
        centerLocked = true;   // aim from odometry right away
        seeded = true;         // first vision frame may replace the seed, even with 1-2 tags
        everSeen = false;
        lastNote = "seeded";
    }

    private void updateTurret(Telemetry t) {
        double now = clock.milliseconds();
        double dt = (now - lastTurretMs) / 1000.0;
        lastTurretMs = now;
        if (dt <= 0) return;

        maxPowerNow = TURRET_MAX_POWER;
        double rx = pose.getX(DistanceUnit.MM);
        double ry = pose.getY(DistanceUnit.MM);
        double rh = pose.getHeading(AngleUnit.RADIANS);
        double omega = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

        readTurret();

        // ---------- VISION ----------
        LLResult r = limelight.getLatestResult();
        if (r != null) {
            double staleness = r.getStaleness();
            boolean newFrame = staleness < lastStaleness;
            lastStaleness = staleness;
            if (newFrame && r.isValid()) {
                double agoMs = staleness + r.getCaptureLatency() + r.getTargetingLatency() + EXTRA_LATENCY_MS;
                int iPose = lookup(now - agoMs);
                int iPan  = lookup(now - agoMs - PAN_LAG_MS);
                if (iPose >= 0 && iPan >= 0) processFrame(r, iPose, iPan, now);
            }
        }

        // ---------- AIM ----------
        boolean visible = now - lastSeenMs < VISION_TIMEOUT_MS;
        aiming = haveCenter && (centerLocked || visible);
        if (aiming) {
            double hAhead = rh + omega * TURRET_LOOKAHEAD_MS / 1000.0;
            double mx = rx + MOUNT_X_MM * Math.cos(rh) - MOUNT_Y_MM * Math.sin(rh);
            double my = ry + MOUNT_X_MM * Math.sin(rh) + MOUNT_Y_MM * Math.cos(rh);
            desired = Math.toDegrees(wrap(Math.atan2(cy - my, cx - mx) - hAhead));
        }

        double dDes = AngleUnit.normalizeDegrees(desired - lastDesired);
        lastDesired = desired;
        double rawVel = (!aiming || Math.abs(dDes) > FF_JUMP_DEG) ? 0 : dDes / dt;
        targetVel += FF_ALPHA * (rawVel - targetVel);

        target = pickReachable(desired);
        saturated = Math.abs(AngleUnit.normalizeDegrees(desired - target)) > 0.5;

        driveTurret(dt);
        record(now, rx, ry, rh, rawDeg);

        String mode = !haveCenter ? "NO CENTER"
                : seeded ? (visible ? "SEED + VISION" : "SEED (no tags yet)")
                  : visible ? "LOCKED + VISION" : "ODOMETRY LOCK";
        t.addData("Turret mode", mode);
        t.addData("Tags / note", "%d  (%s)", lastCnt, lastNote);
        t.addData("Pan desired / target / actual", "%.1f / %.1f / %.1f", desired, target, deg);
        t.addData("Center (mm)", "x %.0f  y %.0f", cx, cy);
        t.addData("Turret SATURATED", saturated);
        t.addData("Pot V / ok", "%.3f / %b", v, potOk);
    }

    private void processFrame(LLResult r, int iPose, int iPan, double now) {
        double h = hH[iPose];
        double panR = Math.toRadians(hPan[iPan]);
        double[] off = {-OFF_OUTER_MM, -OFF_INNER_MM, OFF_INNER_MM, OFF_OUTER_MM};

        double mx = hX[iPose] + MOUNT_X_MM * Math.cos(h) - MOUNT_Y_MM * Math.sin(h);
        double my = hY[iPose] + MOUNT_X_MM * Math.sin(h) + MOUNT_Y_MM * Math.cos(h);
        double camX = mx + CAM_FWD_MM * Math.cos(h + panR);
        double camY = my + CAM_FWD_MM * Math.sin(h + panR);

        boolean[] seen = new boolean[N];
        double[] bearing = new double[N], horiz = new double[N];
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
            seen[i] = true;
            cnt++;
        }
        if (cnt == 0) return;

        int first = -1;
        for (int i = 0; i < N; i++) if (seen[i]) { first = i; break; }
        double ref = bearing[first];

        double[] rel = new double[N];
        double sumS = 0, sumRel = 0, sumH = 0;
        for (int i = 0; i < N; i++) {
            if (!seen[i]) continue;
            rel[i] = wrap(bearing[i] - ref);
            sumS += off[i];
            sumRel += rel[i];
            sumH += horiz[i];
        }
        double sm = sumS / cnt, rm = sumRel / cnt, hm = sumH / cnt;

        lastSeenMs = now;
        lastCnt = cnt;
        everSeen = true;

        if (cnt >= 2) {
            double num = 0, den = 0, numH = 0;
            for (int i = 0; i < N; i++) {
                if (!seen[i]) continue;
                num += (off[i] - sm) * (rel[i] - rm);
                numH += (off[i] - sm) * (horiz[i] - hm);
                den += (off[i] - sm) * (off[i] - sm);
            }
            double k = num / den;
            double c = clamp(-k * hm, -1, 1);
            double aC = ref + rm - sm * k;

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
            lastNote = uAssumed ? "single, dir ASSUMED" : "single, dir from state";
        }

        double sx = 0, sy = 0;
        for (int i = 0; i < N; i++) {
            if (!seen[i]) continue;
            double tX = camX + horiz[i] * Math.cos(bearing[i]);
            double tY = camY + horiz[i] * Math.sin(bearing[i]);
            sx += tX - off[i] * ux;
            sy += tY - off[i] * uy;
        }
        double ax = sx / cnt, ay = sy / cnt;

        boolean strong = cnt >= MIN_TAGS_FOR_LOCK;
        double jump = haveCenter ? Math.hypot(ax - cx, ay - cy) : 0;

        if (!haveCenter || seeded) {
            cx = ax;                     // first real measurement replaces the seed
            cy = ay;
            haveCenter = true;
            seeded = false;
            lastNote += ", seed replaced";
        } else if (jump > JUMP_RESET_MM) {
            if (centerLocked && !strong) {
                lastNote += ", jump REJECTED (weak frame)";
            } else {
                cx = ax;
                cy = ay;
                uAssumed = true;
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

    private void readTurret() {
        double raw = pot.getVoltage();
        potOk = raw > 0.03 && raw < 3.25;
        if (!potOk) return;
        v += FILTER_ALPHA * (raw - v);
        rawDeg = turretDeg(raw);
        deg = turretDeg(v);
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

        motorPower = LATERAL_IS_LEFT ? power : -power;
        if ((v >= V_LIMIT_MAX && motorPower > 0) || (v <= V_LIMIT_MIN && motorPower < 0)) motorPower = 0;
        if (!potOk) motorPower = 0;

        turret.setPower(motorPower);
    }

    private double turretDeg(double volts) {
        double p  = potAngleDeg(volts);
        double pC = potAngleDeg(V_CENTER);
        double old;
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

    private static double potAngleDeg(double volts) {
        if (volts < 1e-3) return 270.0;
        double b = 270.0 * volts + 445.5;
        double c = 120285.0 - 36450.0 * volts;
        return (b - Math.sqrt(b * b - 4.0 * volts * c)) / (2.0 * volts);
    }

    private void record(double tt, double x, double y, double h, double panDeg) {
        hT[hIdx] = tt; hX[hIdx] = x; hY[hIdx] = y; hH[hIdx] = h; hPan[hIdx] = panDeg;
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

    // =========================================================
    //                    UTILITARE
    // =========================================================

    public void refreshPose() {
        pinpoint.update();
        pose = pinpoint.getPosition();
    }

    /** Stops drive only (used between moves). */
    public void stop() {
        stanga.setPower(0);
        dreapta.setPower(0);
    }

    /** OpMode stop(): everything off. */
    public void stopAll() {
        stop();
        stopShooter();
        turret.setPower(0);
        limelight.stop();
    }

    public void resetPose() {
        pinpoint.resetPosAndIMU();
    }

    public Pose2D getPose() { return pose; }
    public double getX() { return pose == null ? 0 : pose.getX(DistanceUnit.MM); }
    public double getY() { return pose == null ? 0 : pose.getY(DistanceUnit.MM); }
    public double getHeading() { return pose == null ? 0 : pose.getHeading(DEGREES); }
    public boolean getStatusXY() { return doneXY; }
    public boolean getStatusHeading() { return doneHeading; }

    private double headingRate() {
        return pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
    }

    private double headingPower(double errorDeg) {
        double raw = Math.tanh(Math.toRadians(errorDeg) * HEADING_TANH_SCALE) * HEADING_KP;
        double power = (raw + Math.copySign(HEADING_KS, raw) - HEADING_KD * headingRate())
                * NOMINAL_VOLTAGE / voltageSensor.getVoltage();
        return Range.clip(power, -HEADING_MAX_POWER, HEADING_MAX_POWER);
    }

    private void setTurn(double power) {
        stanga.setPower(-power);
        dreapta.setPower(power);
    }

    private void setDriveTurn(double drive, double turn) {
        double left = drive - turn;
        double right = drive + turn;
        double maxMag = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)));
        stanga.setPower(left / maxMag);
        dreapta.setPower(right / maxMag);
    }

    private static double[] rotate(double forward, double left, double headingDeg) {
        double h = Math.toRadians(headingDeg);
        double cos = Math.cos(h);
        double sin = Math.sin(h);
        return new double[]{
                forward * cos - left * sin,
                forward * sin + left * cos
        };
    }

    private static double totalTurn(double heading, double lineHeading, double finalHeading) {
        return Math.abs(normalizeAngle(lineHeading - heading))
                + Math.abs(normalizeAngle(finalHeading - lineHeading));
    }

    private static double clamp(double x, double min, double max) {
        return Math.max(min, Math.min(max, x));
    }

    private static double wrap(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }

    private static double normalizeAngle(double angleDeg) {
        while (angleDeg > 180) angleDeg -= 360;
        while (angleDeg < -180) angleDeg += 360;
        return angleDeg;
    }
}