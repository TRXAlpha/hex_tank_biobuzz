package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
@TeleOp(name="teleop driving sag", group = "tank")
public class teleopSagDriving extends OpMode {

    public static double SAG_DROP = 1.5;   // volts below rest voltage = sag
    public static double STALL_TIME = 0.3; // seconds without movement = stalled
    public static double HOLD_TIME = 1.0;  // seconds motors stop

    private DcMotor stanga, dreapta;
    private VoltageSensor battery;
    private int lastL, lastR;
    private double restVolts = 12.0;
    private double stillSince = 0;
    private double stopUntil = 0;

    @Override
    public void init() {
        stanga = hardwareMap.get(DcMotor.class, "stanga");
        dreapta = hardwareMap.get(DcMotor.class, "dreapta");
        stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        battery = hardwareMap.voltageSensor.iterator().next();
        lastL = stanga.getCurrentPosition();
        lastR = dreapta.getCurrentPosition();
    }

    @Override
    public void loop() {
        double now = getRuntime();

        double forward = gamepad1.right_stick_x;
        double turn = -gamepad1.left_stick_y;
        double putereStanga = forward + turn;
        double putereDreapta = forward - turn;
        boolean driving = Math.abs(putereStanga) > 0.1 || Math.abs(putereDreapta) > 0.1;

        double volts = battery.getVoltage();
        int l = stanga.getCurrentPosition();
        int r = dreapta.getCurrentPosition();
        int moved = Math.abs(l - lastL) + Math.abs(r - lastR);
        lastL = l;
        lastR = r;

        if (!driving) restVolts = volts;             // battery voltage at rest
        if (!driving || moved > 3) stillSince = now; // ignore tiny encoder jitter

        double drop = restVolts - volts;
        boolean stalled = driving && now - stillSince >= STALL_TIME;

        // THE if clause: stalled and sagging = stop for HOLD_TIME
        if (stalled && drop >= SAG_DROP) {
            stopUntil = now + HOLD_TIME;
        }

        if (now < stopUntil) {
            stanga.setPower(0);
            dreapta.setPower(0);
        } else {
            stanga.setPower(putereStanga);
            dreapta.setPower(putereDreapta);
        }

        telemetry.addData("Stalled", stalled);
        telemetry.addData("Drop (V)", "%.2f", drop);
        telemetry.addData("Stop active", now < stopUntil);
        telemetry.update();
    }
}