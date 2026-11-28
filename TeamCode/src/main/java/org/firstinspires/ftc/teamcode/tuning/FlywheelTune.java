package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Calib;
import org.firstinspires.ftc.teamcode.Flywheel;

/*
 * FLYWHEEL TUNE - tune the flywheel LIVE on FTC Dashboard. Every change works at once.
 *
 *   Dashboard > Config > Calib:     SHOT_SPEED, KP, KS, KV
 *   Dashboard > Config > Flywheel:  V_CAP, RAMP_TPS_PER_S, MIN_POWER, SPINUP_HOLD_MS, SPUN_DOWN, READY_BAND
 *   Dashboard > Graph:              vel, target, ramped target, power
 *
 *   CROSS    flywheel on / off (watch the spin-up on the graph)
 *   R2 hold  shoot (feeds when at speed)
 *   SQUARE   unjam (intake + stagiu reverse)
 *   CIRCLE   SAVE Calib values to the robot (teleop, autos, calibration use them)
 *
 * Overshoot at spin-up: lower KP, or set RAMP_TPS_PER_S (try 1500-3000), or MIN_POWER -0.2 (brakes).
 * Shots long: lower SHOT_SPEED. Shots short: raise it. Then CIRCLE.
 * Flywheel values (V_CAP, RAMP, MIN_POWER, ...) are NOT saved by CIRCLE: write the good ones in Flywheel.java.
 */
//@TeleOp(name = "FLYWHEEL TUNE", group = "tuning")
public class FlywheelTune extends OpMode {
    private Flywheel fw;
    private DcMotor stagiu, intake;
    private VoltageSensor bat;
    private boolean pCross, pCircle, wasFeeding;
    private double peak = 0;
    private String msg = "", shots = "";

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Calib.load();
        fw = new Flywheel(hardwareMap);
        stagiu = hardwareMap.dcMotor.get("stagiu");
        stagiu.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.dcMotor.get("intake");
        for (DcMotor m : new DcMotor[]{stagiu, intake}) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bat = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void loop() {
        if (gamepad1.cross && !pCross) { fw.setOn(!fw.isOn()); peak = 0; shots = ""; }
        pCross = gamepad1.cross;
        if (gamepad1.circle && !pCircle) { Calib.save(); msg = String.format("SAVED: speed %.0f  KP %.4f  KS %.3f  KV %.6f",
                Calib.SHOT_SPEED, Calib.KP, Calib.KS, Calib.KV); }
        pCircle = gamepad1.circle;

        fw.setTarget(Calib.SHOT_SPEED);
        fw.update(bat.getVoltage());
        if (fw.isOn()) peak = Math.max(peak, fw.getVelocity());

        boolean feed = gamepad1.right_trigger > 0.5 && fw.isOn() && fw.isAtSpeed();
        if (gamepad1.square)  { intake.setPower(-1); stagiu.setPower(-1); }
        else                  { intake.setPower(feed ? 1 : 0); stagiu.setPower(feed ? 1 : 0); }
        if (feed && !wasFeeding) shots = String.format("%.1f%%  ", fw.getVelocity() / Calib.SHOT_SPEED * 100) + shots;
        if (shots.length() > 60) shots = shots.substring(0, 60);
        wasFeeding = feed;

        telemetry.addLine("CROSS on/off, R2 shoot, CIRCLE save. Change values on Dashboard > Config.");
        if (!msg.isEmpty()) telemetry.addLine(msg);
        telemetry.addData("vel", fw.getVelocity());              // numbers -> Dashboard graph
        telemetry.addData("target", Calib.SHOT_SPEED);
        telemetry.addData("ramped target", fw.getRampedTarget());
        telemetry.addData("power", fw.getPower());
        telemetry.addData("Overshoot since CROSS", "%.1f %%", fw.isOn() && Calib.SHOT_SPEED > 0 ? (peak / Calib.SHOT_SPEED - 1) * 100 : 0);
        telemetry.addData("Ready", fw.isAtSpeed() ? "YES" : "no");
        telemetry.addData("Shots (speed at feed)", shots);
        telemetry.addData("Battery", "%.2f V", fw.getVolts());
        telemetry.addData("KP / KS / KV", "%.4f / %.3f / %.6f", Calib.KP, Calib.KS, Calib.KV);
        telemetry.addData("V_CAP / RAMP / MIN_POWER", "%.1f / %.0f / %.2f", Flywheel.V_CAP, Flywheel.RAMP_TPS_PER_S, Flywheel.MIN_POWER);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (fw != null) fw.stop();
        if (stagiu != null) { stagiu.setPower(0); intake.setPower(0); }
    }
}
