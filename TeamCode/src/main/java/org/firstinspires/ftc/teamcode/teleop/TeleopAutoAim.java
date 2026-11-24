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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Flywheel;

/*
 * TELEOP AUTO-AIM. The robot aims, picks the speed and shoots by itself. The driver drives and collects.
 *
 * DRIVER (PS5):
 *   Left stick Y + right stick X   drive
 *   L1 hold        intake
 *   (automatic)    shoots when: 2+ tags seen, turret on target, flywheel at speed, distance in range
 *   R2 hold        FORCE shot (waits only for flywheel speed)
 *   Circle hold    unjam (intake + stagiu reverse)
 *   Square         flywheel on/off (on at start)
 * INIT: D-pad LEFT = RED, D-pad RIGHT = BLUE.
 *
 * HOW IT AIMS, every new camera frame:
 *   1. Keep only our alliance's tags. Use the CELL with the most tags in view.
 *   2. Each tag -> point on the floor, relative to the camera (from tx, ty and range).
 *   3. CELL center = average of (tag point - tag offset along the row). Works with 1 to 4 tags.
 *   4. Save the direction to the center as a FIELD angle (IMU heading + turret angle + camera angle).
 * Between frames, and when the tags are lost, the turret holds that field angle with the IMU heading.
 * The distance turret -> CELL center picks the flywheel speed from the Flywheel table.
 * No odometry position is used: only the IMU heading, which does not drift much.
 *
 * BEFORE THE MEET: Limelight pipeline tag size = 82.55 mm (3.25 in, game manual 9.9).
 * Then check: robot 130 cm from the tags (tape) -> screen "dist" must read ~130.
 */
@Config
@TeleOp(name = "TELEOP AUTO-AIM", group = "turela")
public class TeleopAutoAim extends OpMode {

    // ---------- camera (on the turret) ----------
    public static int PIPELINE = 2;
    public static double CAM_PITCH_DEG = 25;     // lens up from horizontal
    public static double CAM_FWD_MM = 30;        // lens ahead of the turret pivot
    public static double DIST_SCALE = 1.0;       // tape / screen dist. Only if the tag size can't be fixed.
    public static double TAG_OUTER_MM = 165.1, TAG_INNER_MM = 69.85;   // tag centers from cluster center
    public static double LOST_MS = 300;          // tags count as "seen" for this long after the last frame
    public static double DIST_ALPHA = 0.3;       // distance smoothing (1 = none)

    // ---------- turret (pot), same values as teleopPerefect ----------
    public static double V_FRONT = 0.663, V_CENTER = 1.030, V_BACK = 1.500;
    public static double V_LIMIT_MIN = 0.5, V_LIMIT_MAX = 1.75;
    public static boolean LATERAL_IS_LEFT = false, MOTOR_REVERSED = false;
    public static double MAX_POWER = 1, kP = 0.006, kD = 0.0002, kS = 0.2, kV = 0.0015;
    public static double TOLERANCE_DEG = 1.5, FILTER_ALPHA = 0.4;

    // ---------- shooting ----------
    public static double AIM_TOL_DEG = 3;
    public static int FIRE_MIN_TAGS = 2;
    public static boolean SPEED_FROM_DISTANCE = true;   // false = always FIXED_SPEED, no distance check
    public static double FIXED_SPEED = 1300;

    private DcMotor stanga, dreapta, turret, intake, stagiu;
    private AnalogInput pot;
    private Limelight3A ll;
    private GoBildaPinpointDriver imu;
    private Flywheel fw;
    private boolean red = false, flyFile, pSquare;

    // aim
    private boolean haveAim = false;
    private double aimField = 0;          // rad, field direction turret -> CELL center
    private double dist = 0;              // cm, turret pivot -> CELL center
    private int tags = 0;
    private double lastSeen = -1e9, lastStale = 1e9;

    // turret
    private double v, deg, rawDeg, lastDeg, target, minDeg, maxDeg;
    private boolean potOk = true, saturated = false;

    // short history (time, heading, turret) so a camera frame uses the pose from when it was taken
    private static final int H = 64;
    private final double[] hT = new double[H], hHead = new double[H], hPan = new double[H];
    private int hI = 0;

    private final ElapsedTime clock = new ElapsedTime();
    private double lastMs = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
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

        imu = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        imu.setOffsets(-44.3, -6.6, DistanceUnit.MM);
        imu.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        imu.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        imu.resetPosAndIMU();                // keep the robot still in INIT

        flyFile = Flywheel.load();
        fw = new Flywheel(hardwareMap);      // nothing moves in INIT

        double a = turretDeg(V_LIMIT_MIN), b = turretDeg(V_LIMIT_MAX);
        minDeg = Math.min(a, b);
        maxDeg = Math.max(a, b);
        v = pot.getVoltage();
        deg = rawDeg = lastDeg = target = turretDeg(v);
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_left) red = true;
        if (gamepad1.dpad_right) red = false;
        telemetry.addData("ALLIANCE (D-pad L = RED, R = BLUE)", red ? "RED" : "BLUE");
        telemetry.addData("Flywheel values", flyFile ? "from tuner" : "BUILT-IN table");
        telemetry.addData("Battery", "%.2f V%s", fw.getVolts(), fw.getVolts() < 12.3 ? "  <- CHANGE IT" : "");
        telemetry.addLine("Limelight tag size must be 82.55 mm!");
        telemetry.update();
    }

    @Override
    public void start() {
        fw.setOn(true);
        clock.reset();
        lastMs = 0;
    }

    @Override
    public void loop() {
        double now = clock.milliseconds(), dt = (now - lastMs) / 1000.0;
        lastMs = now;
        if (dt <= 0) return;

        // ---------- drive ----------
        double fwd = gamepad1.left_stick_y, turn = -gamepad1.right_stick_x;
        double m = Math.max(1, Math.abs(fwd) + Math.abs(turn));
        stanga.setPower((fwd + turn) / m);
        dreapta.setPower((fwd - turn) / m);

        // ---------- sensors ----------
        imu.update();
        double head = imu.getPosition().getHeading(AngleUnit.RADIANS);
        double omega = imu.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        readTurret();
        hT[hI] = now; hHead[hI] = head; hPan[hI] = rawDeg;
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

        // ---------- turret: hold the field angle to the CELL ----------
        double tv = 0;
        if (haveAim) {
            double want = AngleUnit.normalizeDegrees(Math.toDegrees(aimField - head));
            target = reachable(want);
            saturated = Math.abs(AngleUnit.normalizeDegrees(want - target)) > 0.5;
            if (!saturated) tv = -Math.toDegrees(omega);   // turn against the robot's rotation
        }
        driveTurret(dt, tv);

        // ---------- flywheel ----------
        if (gamepad1.square && !pSquare) fw.setOn(!fw.isOn());
        pSquare = gamepad1.square;
        boolean force = gamepad1.right_trigger > 0.5;
        if (force) fw.setOn(true);
        fw.setTarget(SPEED_FROM_DISTANCE && haveAim ? Flywheel.speedForDistance(dist) : FIXED_SPEED);
        fw.update();

        // ---------- auto-fire decision ----------
        boolean onTarget = haveAim && !saturated && Math.abs(target - deg) <= AIM_TOL_DEG;
        boolean ok = false;
        String why;
        if (!fw.isOn())                                                why = "flywheel OFF (Square)";
        else if (tags < FIRE_MIN_TAGS)                                 why = "looking for tags";
        else if (SPEED_FROM_DISTANCE && dist < Flywheel.MIN_SHOT_CM)   why = "TOO CLOSE - back up";
        else if (SPEED_FROM_DISTANCE && dist > Flywheel.MAX_SHOT_CM)   why = "TOO FAR - get closer";
        else if (saturated)                                            why = "turret at limit - turn robot";
        else if (!onTarget)                                            why = "aiming";
        else if (!fw.isAtSpeed())                                      why = "speeding up";
        else { why = "FIRE"; ok = true; }
        boolean feed = fw.isAtSpeed() && (ok || force);

        // ---------- intake + stagiu (one write each per loop) ----------
        if (gamepad1.circle)  { intake.setPower(-1); stagiu.setPower(-1); }
        else if (feed)        { intake.setPower(1);  stagiu.setPower(1); }
        else                  { intake.setPower(gamepad1.left_bumper ? 1 : 0); stagiu.setPower(0); }

        // ---------- screen ----------
        telemetry.addData("SHOOT", force ? "FORCED" : why);
        telemetry.addData("Tags / dist", "%d / %.0f cm", tags, dist);
        telemetry.addData("Speed", "%.0f / %.0f", fw.getVelocity(), fw.getTarget());
        telemetry.addData("Turret", "%.1f -> %.1f deg%s", deg, target, potOk ? "" : "  POT ERROR");
        telemetry.addData("Battery", "%.2f V%s", fw.getVolts(), fw.getVolts() < 10 ? "  LOW" : "");
        telemetry.addData("Loop", "%.0f ms", dt * 1000);
        telemetry.update();
    }

    // ===================== vision =====================

    /** One camera frame -> CELL center -> aimField + dist. tFrame = when the picture was taken. */
    private void see(LLResult r, double tFrame) {
        int j = (hI - 1 + H) % H;                                   // newest history entry
        for (int s = 0; s < H - 1 && hT[j] > tFrame; s++) j = (j - 1 + H) % H;

        int[] base = red ? new int[]{30, 34} : new int[]{38, 42};    // game manual 9.9: our 2 CELLS
        double[] off = {-TAG_OUTER_MM, -TAG_INNER_MM, TAG_INNER_MM, TAG_OUTER_MM};
        double[][] px = new double[2][4], py = new double[2][4];
        boolean[][] has = new boolean[2][4];
        int[] n = new int[2];

        for (LLResultTypes.FiducialResult f : r.getFiducialResults()) {
            int id = f.getFiducialId();
            int c = id - base[0] >= 0 && id - base[0] < 4 ? 0 : id - base[1] >= 0 && id - base[1] < 4 ? 1 : -1;
            if (c < 0 || has[c][id - base[c]]) continue;
            int i = id - base[c];
            Position p = f.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.MM);
            double range = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z) * DIST_SCALE;
            if (range < 200 || range > 6000) continue;              // bad detection
            double horiz = range * Math.cos(Math.toRadians(CAM_PITCH_DEG + f.getTargetYDegrees()));
            double b = -Math.toRadians(f.getTargetXDegrees());     // tx is + to the right -> + = left
            px[c][i] = horiz * Math.cos(b);                         // forward from the lens
            py[c][i] = horiz * Math.sin(b);                         // left of the lens
            has[c][i] = true;
            n[c]++;
        }
        int c = n[0] >= n[1] ? 0 : 1;
        if (n[c] == 0) return;

        // row direction (tag 0 -> tag 3): from the two outermost tags seen, else assume we face the row
        int lo = -1, hi = -1;
        for (int i = 0; i < 4; i++) if (has[c][i]) { if (lo < 0) lo = i; hi = i; }
        double ux = 0, uy = -1;
        if (hi > lo) {
            ux = px[c][hi] - px[c][lo];
            uy = py[c][hi] - py[c][lo];
            double len = Math.hypot(ux, uy);
            ux /= len;
            uy /= len;
        }
        double cx = 0, cy = 0;
        for (int i = 0; i < 4; i++) {
            if (!has[c][i]) continue;
            cx += px[c][i] - off[i] * ux;
            cy += py[c][i] - off[i] * uy;
        }
        cx = cx / n[c] + CAM_FWD_MM;                                // from the turret pivot
        cy = cy / n[c];

        aimField = wrap(hHead[j] + Math.toRadians(hPan[j]) + Math.atan2(cy, cx));
        double d = Math.hypot(cx, cy) / 10.0;
        dist = haveAim ? dist + DIST_ALPHA * (d - dist) : d;
        haveAim = true;
        tags = n[c];
        lastSeen = clock.milliseconds();
    }

    // ===================== turret =====================

    private void readTurret() {
        double raw = pot.getVoltage();
        potOk = raw > 0.03 && raw < 3.25;
        if (!potOk) return;
        v += FILTER_ALPHA * (raw - v);
        rawDeg = turretDeg(raw);     // unfiltered -> vision history
        deg = turretDeg(v);          // filtered -> control
    }

    private void driveTurret(double dt, double tv) {
        double err = target - deg;
        double vel = (deg - lastDeg) / dt;
        lastDeg = deg;
        double p = kV * tv;
        if (Math.abs(err) > TOLERANCE_DEG) p += kP * err + kD * (tv - vel) + Math.copySign(kS, err);
        p = Math.max(-MAX_POWER, Math.min(MAX_POWER, p));
        p = LATERAL_IS_LEFT ? p : -p;                               // deg space -> "+ raises pot V"
        if ((v >= V_LIMIT_MAX && p > 0) || (v <= V_LIMIT_MIN && p < 0) || !potOk) p = 0;
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

    /** Pot voltage -> turret deg, robot frame (0 front, CCW +). */
    private double turretDeg(double volts) {
        double p = potAngleDeg(volts), pC = potAngleDeg(V_CENTER);
        double old = p <= pC ? (p - pC) / (pC - potAngleDeg(V_BACK)) * 90.0
                             : (p - pC) / (potAngleDeg(V_FRONT) - pC) * 90.0;
        double d = 90.0 - old;
        return LATERAL_IS_LEFT ? d : -d;
    }

    /** REV pot on the hub (not linear): 3.3 V -> 0, 0 V -> 270 pot deg. */
    private static double potAngleDeg(double volts) {
        if (volts < 1e-3) return 270.0;
        double b = 270.0 * volts + 445.5, c = 120285.0 - 36450.0 * volts;
        return (b - Math.sqrt(b * b - 4.0 * volts * c)) / (2.0 * volts);
    }

    private static double wrap(double a) { return Math.atan2(Math.sin(a), Math.cos(a)); }

    @Override
    public void stop() {
        for (DcMotor mo : new DcMotor[]{stanga, dreapta, intake, stagiu, turret}) if (mo != null) mo.setPower(0);
        if (fw != null) fw.stop();
        if (ll != null) ll.stop();
    }
}
