package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/*
 * Flywheel (turelaD + turelaS) with BATTERY-COMPENSATED speed control. Same class in calibration, teleop and auto.
 *
 *   power = (KS + KV * target) * 12 / batteryVolts  +  KP * (target - speed)
 *
 * KS, KV, KP and SHOT_SPEED live in Calib (ROBOT CALIBRATION measures them).
 *
 * Usage:
 *   Calib.load();                            // once, in init
 *   Flywheel fw = new Flywheel(hardwareMap);
 *   fw.setTarget(Calib.SHOT_SPEED);
 *   fw.update(volts);                        // every loop (or update() to read the battery itself)
 *   if (fw.isAtSpeed()) ... feed ...
 */
public class Flywheel {
    // ---- feed gating ----
    public static double READY_BAND = 0.01;  // "at speed" = within +/-1 %
    public static double SHOT_DIP = 0.025;   // below 97.5 % = ball left / not ready
    public static double OVER_BAND = 0.02;   // above 102 % = too fast

    public static final double NOMINAL_V = 12.0;

    private final DcMotorEx d, s;
    private final VoltageSensor battery;
    private double target = 0, vel = 0, volts = NOMINAL_V, power = 0;
    private long lastVoltNs = 0;                 // own battery read: every 100 ms (slow read)
    private boolean on = false, atSpeed = false;

    public Flywheel(HardwareMap hw) {
        d = hw.get(DcMotorEx.class, "turelaD");
        s = hw.get(DcMotorEx.class, "turelaS");
        d.setDirection(DcMotorSimple.Direction.FORWARD);
        s.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotorEx m : new DcMotorEx[]{d, s}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   // we do the control, not the hub
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m.setPower(0);
        }
        battery = hw.voltageSensor.iterator().next();
        volts = battery.getVoltage();
    }

    // ---------------- control ----------------

    public void setTarget(double tps) { target = Math.max(0, tps); }
    public double getTarget() { return target; }
    public void setOn(boolean on) { this.on = on; }
    public boolean isOn() { return on; }

    /** Call once per loop. Reads the battery itself (every 100 ms). */
    public void update() {
        long now = System.nanoTime();
        if (now - lastVoltNs >= 100_000_000L) {
            lastVoltNs = now;
            double v = battery.getVoltage();
            if (v > 5) volts += 0.5 * (v - volts);   // smooth the noise, ignore bad reads
        }
        control();
    }

    /** Call once per loop with the battery volts already read this loop (Power.read()). */
    public void update(double batteryVolts) {
        if (batteryVolts > 5) volts = batteryVolts;
        control();
    }

    private void control() {
        vel = (d.getVelocity() + s.getVelocity()) / 2.0;
        if (!on || target <= 0) {
            power = 0;
            atSpeed = false;
        } else {
            power = (Calib.KS + Calib.KV * target) * NOMINAL_V / volts + Calib.KP * (target - vel);
            power = Math.max(0, Math.min(1, power));
            double lo = target * (1 - SHOT_DIP), hi = target * (1 + OVER_BAND);
            if (vel < lo || vel > hi) atSpeed = false;
            else if (Math.abs(vel - target) <= target * READY_BAND) atSpeed = true;
        }
        d.setPower(power);
        s.setPower(power);
    }

    /** Raw power, no control (calibration only). */
    public void setRawPower(double p) {
        vel = (d.getVelocity() + s.getVelocity()) / 2.0;
        power = p;
        atSpeed = false;
        d.setPower(p);
        s.setPower(p);
    }

    /** true = BRAKE at zero power (calibration fake ball), false = FLOAT (normal). */
    public void setBrake(boolean brake) {
        DcMotor.ZeroPowerBehavior z = brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        d.setZeroPowerBehavior(z);
        s.setZeroPowerBehavior(z);
    }

    public void stop() {
        on = false;
        power = 0;
        d.setPower(0);
        s.setPower(0);
    }

    public double getVelocity() { return vel; }
    public double getVelD() { return d.getVelocity(); }
    public double getVelS() { return s.getVelocity(); }
    public double getVolts() { return volts; }
    public double getPower() { return power; }
    public boolean isAtSpeed() { return atSpeed; }
    public DcMotorEx motorD() { return d; }
    public DcMotorEx motorS() { return s; }

    /** Highest speed the battery can hold right now (keeps 10 % for KP). */
    public double maxSpeedNow() {
        return (0.9 * volts / NOMINAL_V - Calib.KS) / Calib.KV;
    }

    // ---- old API, kept so older OpModes (teleopPerefectRed/Blue) still compile. Speed is fixed now. ----
    public static double MIN_SHOT_CM = 0, MAX_SHOT_CM = 1e9;
    public static boolean load() { return Calib.load(); }
    public static double speedForDistance(double cm) { return Calib.SHOT_SPEED; }
    public static boolean inRange(double cm) { return true; }
}
