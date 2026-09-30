package org.firstinspires.ftc.teamcode.claudiu;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Heading-only controller for 2-wheel tank drive, no motor encoders.
 * Feedback = Pinpoint heading + omega derived from heading (version-safe, no getHeadingVelocity).
 *
 * Loop:  heading error --P--> omega target (deg/s, clamped, slewed)
 *        omega target --kS + kV*w + kP_w*wErr + kI_w*∫wErr--> turn power
 *        turn power * voltage comp -> stanga = -p, dreapta = p * RIGHT_SCALE
 */
@Config
public class TankHeadingHelper {

    // ---------- trim / signs ----------
    public static double RIGHT_SCALE = 1.0;   // trim dreapta vs stanga
    public static double TURN_SIGN = 1.0;     // flip to -1 if robot spins away from target

    // ---------- outer loop: heading error -> omega target ----------
    public static double KP_HEADING = 4.0;        // deg/s per deg error
    public static double MAX_OMEGA_DPS = 180;     // max spin speed
    public static double MIN_OMEGA_DPS = 12;      // floor so we never ask for ~0 outside deadband
    public static double MAX_ALPHA_DPS2 = 600;    // ramp-up limit (deg/s^2), smooths start

    // ---------- inner loop: omega -> power ----------
    public static double KS = 0.18;      // static friction power
    public static double KV = 0.0015;    // power per deg/s  (tune, see notes)
    public static double KP_W = 0.002;   // power per deg/s omega error
    public static double KI_W = 0.0;     // start at 0
    public static double I_CLAMP = 0.15; // max power contribution from I
    public static double MAX_POWER = 0.9;
    public static double NOMINAL_VOLTAGE = 12.5;

    // ---------- done / hold ----------
    public static double DEADBAND_DEG = 1.0;
    public static double SETTLE_DPS = 10;
    public static double REENGAGE_DEG = 2.5;  // after done, resume if error exceeds this
    public static double RATE_FILTER = 0.5;   // 0..1, weight of newest omega sample

    private final DcMotor stanga, dreapta;
    private final GoBildaPinpointDriver pinpoint;
    private final VoltageSensor voltageSensor;
    private final ElapsedTime timer = new ElapsedTime();

    private Pose2D pose;
    private boolean first = true;
    private boolean done = false;
    private double lastHeading, rate, wCmd, integral;
    private double startX, startY, maxDriftMm;

    public TankHeadingHelper(DcMotor stanga, DcMotor dreapta,
                             GoBildaPinpointDriver pinpoint, HardwareMap hw) {
        this.stanga = stanga;
        this.dreapta = dreapta;
        this.pinpoint = pinpoint;
        this.voltageSensor = hw.voltageSensor.iterator().next();
    }

    public static TankHeadingHelper fromHardwareMap(HardwareMap hw,
                                                    String leftName, String rightName, String pinpointName,
                                                    double podOffsetX_mm, double podOffsetY_mm) {
        DcMotor l = hw.get(DcMotor.class, leftName);
        DcMotor r = hw.get(DcMotor.class, rightName);

        // same convention as AutoHelper
        l.setDirection(DcMotorSimple.Direction.REVERSE);
        r.setDirection(DcMotorSimple.Direction.FORWARD);
        l.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        l.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // no motor encoders
        r.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        GoBildaPinpointDriver p = hw.get(GoBildaPinpointDriver.class, pinpointName);
        p.setOffsets(podOffsetX_mm, podOffsetY_mm, DistanceUnit.MM);
        p.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        p.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        p.resetPosAndIMU();
        return new TankHeadingHelper(l, r, p, hw);
    }

    /** Call every loop. Returns true when heading is settled (keeps holding afterwards). */
    public boolean update(double targetDeg, Telemetry t) {
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0 || dt > 0.2) dt = 0.02;

        pinpoint.update();
        pose = pinpoint.getPosition();
        double h = pose.getHeading(DEGREES);

        if (first) {
            lastHeading = h;
            rate = 0;
            startX = pose.getX(DistanceUnit.MM);
            startY = pose.getY(DistanceUnit.MM);
            maxDriftMm = 0;
            first = false;
        } else {
            double raw = normalize(h - lastHeading) / dt;
            rate = RATE_FILTER * raw + (1 - RATE_FILTER) * rate;
            lastHeading = h;
        }

        double drift = getDriftMm();
        maxDriftMm = Math.max(maxDriftMm, drift);

        double error = normalize(targetDeg - h);

        // ---- done / hold logic ----
        if (done) {
            if (Math.abs(error) > REENGAGE_DEG) {
                done = false;
            } else {
                stop();
                addTelemetry(t, targetDeg, h, error, 0, 0, drift);
                return true;
            }
        }
        if (Math.abs(error) <= DEADBAND_DEG && Math.abs(rate) <= SETTLE_DPS) {
            done = true;
            integral = 0;
            wCmd = 0;
            stop();
            addTelemetry(t, targetDeg, h, error, 0, 0, drift);
            return true;
        }

        // ---- outer loop: error -> omega target ----
        double wT = 0;
        if (Math.abs(error) > DEADBAND_DEG) {
            wT = Range.clip(KP_HEADING * error, -MAX_OMEGA_DPS, MAX_OMEGA_DPS);
            if (Math.abs(wT) < MIN_OMEGA_DPS) wT = Math.copySign(MIN_OMEGA_DPS, error);
        }

        // slew only when speeding up (decel follows the P term directly)
        if (Math.abs(wT) > Math.abs(wCmd) || Math.signum(wT) != Math.signum(wCmd)) {
            double step = MAX_ALPHA_DPS2 * dt;
            wCmd += Range.clip(wT - wCmd, -step, step);
        } else {
            wCmd = wT;
        }

        // ---- inner loop: omega -> power ----
        double wErr = wCmd - rate;
        if (KI_W > 0) {
            integral += wErr * dt;
            double lim = I_CLAMP / KI_W;
            integral = Range.clip(integral, -lim, lim);
        } else {
            integral = 0;
        }

        double p = KV * wCmd + KP_W * wErr + KI_W * integral;
        if (Math.abs(wCmd) > 1e-6) p += KS * Math.signum(wCmd);

        p *= NOMINAL_VOLTAGE / voltageSensor.getVoltage();
        p = Range.clip(p, -MAX_POWER, MAX_POWER) * TURN_SIGN;

        stanga.setPower(-p);
        dreapta.setPower(p * RIGHT_SCALE);

        addTelemetry(t, targetDeg, h, error, wCmd, p, drift);
        return false;
    }

    private void addTelemetry(Telemetry t, double target, double h, double err,
                              double wT, double p, double drift) {
        if (t == null) return;
        t.addData("target", target);
        t.addData("heading", h);
        t.addData("error", err);
        t.addData("omega meas (dps)", rate);
        t.addData("omega cmd (dps)", wT);
        t.addData("power", p);
        t.addData("done", done);
        t.addData("drift from start (mm)", drift);
        t.addData("max drift (mm)", maxDriftMm);
        t.addData("x (mm)", pose.getX(DistanceUnit.MM));
        t.addData("y (mm)", pose.getY(DistanceUnit.MM));
        t.addData("battery V", voltageSensor.getVoltage());
    }

    // ---------- utils ----------
    public void stop() {
        stanga.setPower(0);
        dreapta.setPower(0);
    }

    /** Zero pose + IMU, clear controller state and drift tracking. */
    public void reset() {
        pinpoint.resetPosAndIMU();
        first = true;
        done = false;
        wCmd = 0;
        integral = 0;
        rate = 0;
        maxDriftMm = 0;
    }

    /** Restart drift measurement from current position without resetting pose. */
    public void resetDrift() {
        if (pose != null) {
            startX = pose.getX(DistanceUnit.MM);
            startY = pose.getY(DistanceUnit.MM);
        }
        maxDriftMm = 0;
    }

    public double getDriftMm() {
        if (pose == null) return 0;
        return Math.hypot(pose.getX(DistanceUnit.MM) - startX, pose.getY(DistanceUnit.MM) - startY);
    }

    private static double normalize(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }
}