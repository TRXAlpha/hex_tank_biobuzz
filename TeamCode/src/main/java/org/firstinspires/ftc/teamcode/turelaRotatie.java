package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@Autonomous(name = "Turret PID Test", group = "Test")
public class turelaRotatie extends LinearOpMode {

    // ---- TUNE THESE ----
    static final String MOTOR_NAME = "turret";
    public static double TARGET_DEGREES = 90;

    static final double TICKS_PER_MOTOR_REV = 28 * 9.0;      // REV HD Hex 9:1
    static final double MOTOR_TURNS_PER_TURRET_TURN = 1.0;   // external ratio

    // PID gains (units: motor power per degree). Tune in this order: kP, kD, kI
   public static  double kP = 0.01;
    public static  double kI = 0.0;
   public  static  double kD = 0.0003;

    static final double MAX_POWER = 0.6;       // speed cap
    static final double MIN_POWER = 0.0;       // raise (e.g. 0.05) if it stalls near target
    static final double TOLERANCE_DEG = 1.0;   // "close enough"
    static final double I_ZONE_DEG = 10;       // integral only active this close to target
    static final double I_MAX = 0.2;           // integral power clamp
    // --------------------

    static final double TICKS_PER_DEGREE =
            (TICKS_PER_MOTOR_REV * MOTOR_TURNS_PER_TURRET_TURN) / 360.0;

    @Override
    public void runOpMode() {
        DcMotor turret = hardwareMap.get(DcMotor.class, MOTOR_NAME);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double integral = 0;
        double lastError = TARGET_DEGREES;
        ElapsedTime timer = new ElapsedTime();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            double dt = timer.seconds();
            timer.reset();
            if (dt <= 0) continue;

            double angle = turret.getCurrentPosition() / TICKS_PER_DEGREE;
            double error = TARGET_DEGREES - angle;

            // I term with anti-windup
            if (Math.abs(error) < I_ZONE_DEG) {
                integral += error * dt;
                integral = clamp(integral, -I_MAX / Math.max(kI, 1e-9), I_MAX / Math.max(kI, 1e-9));
            } else {
                integral = 0;
            }

            double derivative = (error - lastError) / dt;
            lastError = error;

            double power = kP * error + kI * integral + kD * derivative;
            power = clamp(power, -MAX_POWER, MAX_POWER);

            // small push to beat friction, only while outside tolerance
            if (Math.abs(error) > TOLERANCE_DEG && Math.abs(power) < MIN_POWER) {
                power = Math.signum(error) * MIN_POWER;
            }
            if (Math.abs(error) <= TOLERANCE_DEG) {
                power = 0; // holds with BRAKE; PID resumes if it gets pushed out
                integral = 0;
            }

            turret.setPower(power);

            telemetry.addData("Target", TARGET_DEGREES);
            telemetry.addData("Angle", "%.1f", angle);
            telemetry.addData("Error", "%.1f", error);
            telemetry.addData("Power", "%.2f", power);
            telemetry.update();
        }

        turret.setPower(0);
    }

    static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}