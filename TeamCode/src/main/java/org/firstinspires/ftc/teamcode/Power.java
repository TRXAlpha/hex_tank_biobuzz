package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Battery model + current limiting for the DRIVE (turret and flywheel are never limited).
 *
 *   battery volts = Voc - BAT_R * amps
 *   drive amps (one side) = I_DRIVE_FULL / 2 * (power * volts / 12 - wheel speed)   speed follows it (DRIVE_TAU)
 *   (BAT_R, I_DRIVE_FULL, DRIVE_TAU: measured by ROBOT CALIBRATION steps 4 + 5)
 *
 * PREDICT: before new drive powers go out, the extra current they cause is predicted. If the battery would
 *   drop under V_FLOOR, the power change is made smaller. Braking never draws battery current, never limited.
 * LEARN: when the flywheel starts from stop (known current I_FLY_START) and the robot is not driving, the volt
 *   drop gives the resistance of the battery in use NOW (call learnFromFlywheelStart()). Until then: BAT_R.
 * BACKUP: if the battery is still measured under V_FLOOR, drive power is cut fast and comes back slowly.
 *
 * Good battery: big headroom, it never acts -> no lag. Weak battery: no brownout (hub / Limelight reset).
 *
 * Usage, every loop:  power.read();  double[] d = power.drive(left, right, dt);  fw.update(power.volts());
 */
@Config
public class Power {
    public static double V_FLOOR = 9.0;         // keep the battery above this (volts)
    public static double CUT_PER_LOOP = 0.85;   // backup: drive scale *= this every loop under V_FLOOR
    public static double RECOVER_PER_S = 1.5;   // backup: drive scale comes back this fast (per second)
    public static double MARGIN = 1.2;          // predicted amps x this (the measured R and current read ~10-15 % low)

    private final VoltageSensor bat;
    private double v, scale = 1, lastL = 0, lastR = 0, sL = 0, sR = 0, cut = 0;
    private double rLearned = 0, learnRest, learnSum;             // resistance of the battery in use
    private final ElapsedTime learnTimer = new ElapsedTime();
    private boolean learning, learnSpoiled;
    private int learnN;

    public Power(HardwareMap hw) {
        bat = hw.voltageSensor.iterator().next();
        v = bat.getVoltage();
    }

    /** Read the battery once, at the start of every loop. */
    public double read() {
        double r = bat.getVoltage();
        if (r > 3) v = r;
        if (learning) {                                               // flywheel start: average the dip
            if (learnTimer.seconds() < 0.3) { learnSum += v; learnN++; }
            else {
                double amps = Calib.I_FLY_START * learnRest / 12;     // flywheel start current at these volts
                double rNow = learnN > 2 && amps > 1 ? (learnRest - learnSum / learnN) / amps : -1;
                if (!learnSpoiled && rNow > 0 && rNow < 1) rLearned = rNow;
                learning = false;
            }
        }
        return v;
    }

    /** Call in the loop where the flywheel starts from stop, before it gets power. Measures for 0.3 s. */
    public void learnFromFlywheelStart() {
        learnRest = v;
        learnSum = 0;
        learnN = 0;
        learnSpoiled = false;
        learning = true;
        learnTimer.reset();
    }

    public double volts() { return v; }

    /** Battery + wiring resistance used now (ohm): learned at the last flywheel start, else calibrated. */
    public double ohms() { return rLearned > 0 ? rLearned : Calib.BAT_R; }

    /** Part of the last drive request that was removed (0 = none, 0.3 = 30 %). */
    public double cut() { return cut; }

    /** Limit the drive powers (-1..1). Returns {left, right} to send to the motors. */
    public double[] drive(double l, double r, double dt) {
        double reqL = l, reqR = r;
        if (learning && (Math.abs(l) > 0.05 || Math.abs(r) > 0.05 || Math.abs(lastL) > 0.05 || Math.abs(lastR) > 0.05))
            learnSpoiled = true;                                          // driving: the dip is not only the flywheel
        double a = Math.min(1, dt / Math.max(0.02, Calib.DRIVE_TAU));     // wheel speeds follow the power
        sL += (lastL * v / 12 - sL) * a;
        sR += (lastR * v / 12 - sR) * a;

        if (v < V_FLOOR) scale *= CUT_PER_LOOP;                           // backup feedback
        else scale = Math.min(1, scale + RECOVER_PER_S * dt);
        l = Math.max(-scale, Math.min(scale, l));
        r = Math.max(-scale, Math.min(scale, r));

        double head = Math.max(0, v - V_FLOOR) / Math.max(1e-3, ohms() * MARGIN);   // amps we may add
        double nowL = amps(lastL, sL), nowR = amps(lastR, sR);
        double dL = Math.max(0, amps(l, sL) - nowL), dR = Math.max(0, amps(r, sR) - nowR);
        if (dL + dR > head) {
            double k = head / (dL + dR);
            l = toward(lastL, l, sL, nowL + dL * k);
            r = toward(lastR, r, sR, nowR + dR * k);
        }
        double req = Math.abs(reqL) + Math.abs(reqR);
        cut = req > 1e-6 ? Math.max(0, 1 - (Math.abs(l) + Math.abs(r)) / req) : 0;
        lastL = l;
        lastR = r;
        return new double[]{l, r};
    }

    /** Battery amps of one side at power p while the wheel turns at speed s (fraction of free speed at 12 V). */
    private double amps(double p, double s) {
        double e = p * v / 12, f;                                         // applied volts, as a fraction of 12 V
        if (p == 0) f = 0;                                                // brake: no battery current
        else if (Math.signum(e) == Math.signum(s)) f = Math.max(0, Math.abs(e) - Math.abs(s));   // regen -> 0
        else f = Math.abs(e) + Math.abs(s);                               // pushing against the motion
        return f * Calib.I_DRIVE_FULL / 2;
    }

    /** The power closest to want (on the way from last) whose amps stay under maxAmps. */
    private double toward(double last, double want, double s, double maxAmps) {
        double lo = 0, hi = 1;
        for (int i = 0; i < 12; i++) {
            double k = (lo + hi) / 2;
            if (amps(last + k * (want - last), s) <= maxAmps) lo = k; else hi = k;
        }
        return last + lo * (want - last);
    }
}
