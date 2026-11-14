package org.firstinspires.ftc.teamcode.tuning.turela;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Field-locked turret: robot spins, turret keeps pointing the same way on the field.
 *
 * Heading: goBILDA Pinpoint (CCW +). Field 0 = robot forward at init.
 * Turret angle (robot frame, CCW +): 0 = robot front, +-90 = lateral (center), +-180 = back.
 * Feedback: pot only. Feedforward: cancels robot rotation rate.
 */
@Config
@TeleOp(name = "Turela Field Lock", group = "turela")
public class turelaFieldLock extends OpMode {

    // ---------- target ----------
    public static double FIELD_TARGET_DEG = 0;   // set from Dashboard; overwritten at start to "hold current"

    // ---------- pot calibration (your values) ----------
    public static double V_FRONT  = 0.663;   // robot front
    public static double V_CENTER = 1.030;   // lateral
    public static double V_BACK   = 1.500;   // robot back
    public static double V_LIMIT_MIN = 0.5;
    public static double V_LIMIT_MAX = 1.75;
    /** true if the "center" (lateral) position is on the robot's LEFT. Wrong = turret runs WITH the spin. */
    public static boolean LATERAL_IS_LEFT = false;

    // ---------- motor ----------
    public static boolean MOTOR_REVERSED = false;   // keep your tuned value: + power raises pot V
    public static double MAX_POWER = 1;

    // ---------- control (degrees) ----------
    public static double kP = 0.006;     // ~ your kP 1.5 per volt converted
    public static double kD = 0.0002;
    public static double kS = 0.2;
    public static double kV_TURN = 0.0015;  // power per deg/s of robot rotation (full power ~620 deg/s)
    public static double TOLERANCE_DEG = 1.5;
    public static double FILTER_ALPHA = 1;

    // ---------- Pinpoint ----------
    public static String PINPOINT_NAME = "pinpoint";

    DcMotor turret;
    AnalogInput pot;
    GoBildaPinpointDriver pinpoint;

    double v, deg, lastDeg;
    double heading, lastHeading, headingRate;
    double minDeg, maxDeg;
    final ElapsedTime timer = new ElapsedTime();

    // ---------- lifecycle ----------

    @Override
    public void init() {
        turret   = hardwareMap.get(DcMotor.class, "turret");
        pot      = hardwareMap.get(AnalogInput.class, "potTurela");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);

        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint.resetPosAndIMU();   // keep robot still during init
        v = pot.getVoltage();
    }

    @Override
    public void init_loop() {
        pinpoint.update();
        v = pot.getVoltage();
        computeLimits();
        telemetry.addData("Pot V", "%.3f", v);
        telemetry.addData("Turret deg (robot)", "%.1f", turretDeg(v));
        telemetry.addData("Reach", "%.0f .. %.0f", minDeg, maxDeg);
        telemetry.addData("Heading", "%.1f", readHeading());
        telemetry.update();
    }

    @Override
    public void start() {
        turret.setDirection(MOTOR_REVERSED ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
        computeLimits();
        pinpoint.update();
        heading = lastHeading = readHeading();
        v = pot.getVoltage();
        deg = lastDeg = turretDeg(v);
        FIELD_TARGET_DEG = AngleUnit.normalizeDegrees(heading + deg);   // hold current field direction
        timer.reset();
    }

    @Override
    public void loop() {
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) return;

        // --- heading + rotation rate ---
        pinpoint.update();
        heading = readHeading();
        headingRate = AngleUnit.normalizeDegrees(heading - lastHeading) / dt;   // deg/s, CCW +
        lastHeading = heading;

        // --- turret angle from pot ---
        double raw = pot.getVoltage();
        boolean potOk = raw > 0.03 && raw < 3.25;
        if (potOk) v += FILTER_ALPHA * (raw - v);
        deg = turretDeg(v);

        // --- field target -> robot-frame target, inside reachable range ---
        double want = FIELD_TARGET_DEG - heading;
        double target = pickReachable(want);
        boolean reachable = Math.abs(target - wrapNear(want, target)) < 0.5;

        // --- control ---
        double error = target - deg;
        double turretVel = (deg - lastDeg) / dt;
        lastDeg = deg;
        double targetVel = -headingRate;                  // turret must counter-rotate
        double derivative = targetVel - turretVel;        // D on velocity error, no kick from spin

        double power = kV_TURN * targetVel;               // feedforward
        if (Math.abs(error) > TOLERANCE_DEG) {
            power += kP * error + kD * derivative + Math.copySign(kS, error);
        }
        power = clamp(power, -MAX_POWER, MAX_POWER);

        // deg-space -> motor power (+ motor power raises pot V)
        double motorPower = LATERAL_IS_LEFT ? power : -power;

        // safety in voltage space
        if ((v >= V_LIMIT_MAX && motorPower > 0) || (v <= V_LIMIT_MIN && motorPower < 0)) motorPower = 0;
        if (!potOk) motorPower = 0;

        turret.setPower(motorPower);

        telemetry.addData("Heading", "%.1f", heading);
        telemetry.addData("Heading rate", "%.0f deg/s", headingRate);
        telemetry.addData("Field target", "%.1f", FIELD_TARGET_DEG);
        telemetry.addData("Turret target (robot)", "%.1f", target);
        telemetry.addData("Turret deg (robot)", "%.1f", deg);
        telemetry.addData("Error", "%.1f", error);
        telemetry.addData("Reachable", reachable);
        telemetry.addData("Pot V", "%.3f", v);
        telemetry.addData("Motor power", "%.2f", motorPower);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (turret != null) turret.setPower(0);
    }

    // ---------- helpers ----------

    double readHeading() {
        return pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
    }

    /** Pot voltage -> turret deg in robot frame (0 front, +-90 lateral, +-180 back). */
    double turretDeg(double volts) {
        double p  = potAngleDeg(volts);
        double pC = potAngleDeg(V_CENTER);
        double old;   // +90 front, 0 center, -90 back
        if (p <= pC) old = (p - pC) / (pC - potAngleDeg(V_BACK)) * 90.0;
        else         old = (p - pC) / (potAngleDeg(V_FRONT) - pC) * 90.0;
        double d = 90.0 - old;
        return LATERAL_IS_LEFT ? d : -d;
    }

    void computeLimits() {
        double a = turretDeg(V_LIMIT_MIN), b = turretDeg(V_LIMIT_MAX);
        minDeg = Math.min(a, b);
        maxDeg = Math.max(a, b);
    }

    /** Pick the 360-equivalent of 'want' inside [minDeg, maxDeg]; if none, the nearest limit. */
    double pickReachable(double want) {
        want = AngleUnit.normalizeDegrees(want);
        double best = 0, bestDist = Double.MAX_VALUE;
        for (int k = -1; k <= 1; k++) {
            double c = want + 360.0 * k;
            double d = c < minDeg ? minDeg - c : (c > maxDeg ? c - maxDeg : 0);
            if (d < bestDist) { bestDist = d; best = clamp(c, minDeg, maxDeg); }
        }
        return best;
    }

    static double wrapNear(double a, double ref) {
        return ref + AngleUnit.normalizeDegrees(a - ref);
    }

    /** REV pot on FTC hub (nonlinear): 3.3V->0, 1.1V->135, 0V->270 pot deg. */
    static double potAngleDeg(double volts) {
        if (volts < 1e-3) return 270.0;
        double b = 270.0 * volts + 445.5;
        double c = 120285.0 - 36450.0 * volts;
        return (b - Math.sqrt(b * b - 4.0 * volts * c)) / (2.0 * volts);
    }

    static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }
}