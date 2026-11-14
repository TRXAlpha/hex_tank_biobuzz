package org.firstinspires.ftc.teamcode.tuning.turela;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Turret PID directly on pot voltage. Set TARGET_V from FTC Dashboard.
 * Reference: 0.663 front | 1.030 center | 1.500 back | ends 0.460 / 1.790
 */
@Config
@TeleOp(name = "Turela Voltage PID", group = "turela")
public class turelaRotMotor extends OpMode {

    public static double FILTER_ALPHA = 0.4;
    public static double MAX_POWER = 1;
    public static boolean MOTOR_REVERSED = false;
    public static double TARGET_V = 1.03;
    public static double TOLERANCE_V = 0.008;
    public static double V_LIMIT_MAX = 1.75;
    public static double V_LIMIT_MIN = 0.5;
    public static double kD = 0.05;
    public static double kP = 1.5;
    public static double kS = 0.2;

    DcMotor turret;
    AnalogInput pot;
    double v, lastV;
    final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotor.class, "turret");
        pot    = hardwareMap.get(AnalogInput.class, "potTurela");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        v = pot.getVoltage();
    }

    @Override
    public void init_loop() {
        v = pot.getVoltage();
        telemetry.addData("Pot V", "%.3f", v);
        telemetry.update();
    }

    @Override
    public void start() {
        turret.setDirection(MOTOR_REVERSED ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
        v = lastV = pot.getVoltage();
        TARGET_V = clamp(v, V_LIMIT_MIN, V_LIMIT_MAX);  // hold where it is
        timer.reset();
    }

    @Override
    public void loop() {
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) return;

        double raw = pot.getVoltage();
        boolean potOk = raw > 0.03 && raw < 3.25;
        if (potOk) v += FILTER_ALPHA * (raw - v);

        double target = clamp(TARGET_V, V_LIMIT_MIN, V_LIMIT_MAX);
        double error = target - v;

        double derivative = -(v - lastV) / dt;
        lastV = v;

        double power = 0;
        if (Math.abs(error) > TOLERANCE_V) {
            power = kP * error + kD * derivative + Math.copySign(kS, error);
        }
        power = clamp(power, -MAX_POWER, MAX_POWER);

        // never push further past a limit, never move blind
        if ((v >= V_LIMIT_MAX && power > 0) || (v <= V_LIMIT_MIN && power < 0)) power = 0;
        if (!potOk) power = 0;

        turret.setPower(power);

        telemetry.addData("Target V", "%.3f", target);
        telemetry.addData("Pot V", "%.3f", v);
        telemetry.addData("Error V", "%.3f", error);
        telemetry.addData("Power", "%.2f", power);
        telemetry.addData("Pot OK", potOk);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (turret != null) turret.setPower(0);
    }

    static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }
}