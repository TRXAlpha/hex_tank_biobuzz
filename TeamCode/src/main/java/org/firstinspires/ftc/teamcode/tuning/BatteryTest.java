package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * BATTERY TEST - about 8 s. Run it with EVERY battery and compare. Robot does not drive.
 *
 * 1. Rest voltage, everything off.
 * 2. Flywheel full power from stop (big current). Voltage vs current -> resistance of
 *    battery + switch + wires + connectors. Higher = more sag.
 * 3. Intake + stagiu alone -> their current. High current = jam or friction.
 *
 * Take the balls out first.
 */
@TeleOp(name = "Battery Test", group = "tuning")
public class BatteryTest extends LinearOpMode {
    private static final double ROBOT_PEAK_AMPS = 50;   // rough guess: drive + flywheel spin-up + turret + intake

    @Override
    public void runOpMode() {
        DcMotorEx d = hardwareMap.get(DcMotorEx.class, "turelaD");
        DcMotorEx s = hardwareMap.get(DcMotorEx.class, "turelaS");
        DcMotorEx in = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx st = hardwareMap.get(DcMotorEx.class, "stagiu");
        s.setDirection(DcMotorSimple.Direction.REVERSE);
        st.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotorEx m : new DcMotorEx[]{d, s}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        VoltageSensor bat = hardwareMap.voltageSensor.iterator().next();

        telemetry.addLine("BATTERY TEST: take the balls out. Robot stays still. Press START.");
        telemetry.update();
        waitForStart();
        ElapsedTime t = new ElapsedTime();

        // 1. rest voltage
        double rest = 0;
        int n = 0;
        t.reset();
        while (opModeIsActive() && t.seconds() < 0.5) { rest += bat.getVoltage(); n++; }
        rest /= Math.max(1, n);

        // 2. flywheel spin-up: fit V = rest - R * I (least squares over all samples)
        double sI = 0, sV = 0, sII = 0, sIV = 0, minV = rest, maxI = 0;
        n = 0;
        d.setPower(1);
        s.setPower(1);
        t.reset();
        while (opModeIsActive() && t.seconds() < 1.5) {
            double i = d.getCurrent(CurrentUnit.AMPS) + s.getCurrent(CurrentUnit.AMPS);
            double v = bat.getVoltage();
            sI += i; sV += v; sII += i * i; sIV += i * v; n++;
            minV = Math.min(minV, v);
            maxI = Math.max(maxI, i);
        }
        d.setPower(0);
        s.setPower(0);
        double den = n * sII - sI * sI;
        double ohm = den > 1e-6 ? -(n * sIV - sI * sV) / den : 0;
        if (ohm <= 0 && maxI > 1) ohm = (rest - minV) / maxI;        // fallback: simple drop / current

        sleep(2500);                                                 // let the flywheel coast down

        // 3. intake + stagiu current
        in.setPower(1);
        st.setPower(1);
        sleep(500);
        double feedI = 0;
        n = 0;
        t.reset();
        while (opModeIsActive() && t.seconds() < 1.0) {
            feedI += in.getCurrent(CurrentUnit.AMPS) + st.getCurrent(CurrentUnit.AMPS);
            n++;
        }
        feedI /= Math.max(1, n);
        in.setPower(0);
        st.setPower(0);

        double atPeak = rest - ohm * ROBOT_PEAK_AMPS;
        String verdict = atPeak >= 9.5 ? "GOOD - use in matches"
                       : atPeak >= 8.0 ? "WEAK - practice only"
                       : "BAD - do not use";

        while (opModeIsActive()) {
            telemetry.addData("Rest voltage", "%.2f V%s", rest, rest < 12.3 ? "  (low: not charged or old)" : "");
            telemetry.addData("Resistance", "%.0f milliohm  (lower = better)", ohm * 1000);
            telemetry.addData("Flywheel spin-up", "max %.1f A, lowest %.2f V", maxI, minV);
            telemetry.addData("Full robot (~50 A)", "~%.1f V", atPeak);
            telemetry.addData("VERDICT", verdict);
            telemetry.addData("Intake + stagiu", "%.1f A%s", feedI, feedI > 5 ? "  HIGH: jam / friction / bad bearing" : "  ok");
            telemetry.addLine("");
            telemetry.addLine("Write the resistance on the battery with tape. Test all batteries.");
            telemetry.addLine("If ALL batteries are high: check switch, wires, connectors.");
            telemetry.update();
        }
    }
}
