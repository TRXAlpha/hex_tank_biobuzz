package org.firstinspires.ftc.teamcode.limelight.motor;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Config
//@TeleOp(name = "Turela Teleop", group = "turela")
public class turelaRotatie extends LinearOpMode {

    static final String MOTOR_NAME = "turret";

    static final double TICKS_PER_MOTOR_REV = 28 * 9.0;      // REV HD Hex 9:1
    static final double MOTOR_TURNS_PER_TURRET_TURN = 1.0;   // update when gearing is added

    // PID (same gains as the working autonomous test)
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0003;

    public static double MAX_POWER = 0.6;
    public static double TOLERANCE_DEG = 1.0;
    public static double I_ZONE_DEG = 10;
    public static double I_MAX = 0.2;

    // Joystick control
    public static double MAX_DEG_PER_SEC = 120;  // target speed at full stick
    public static double STICK_DEADZONE = 0.05;
    public static double MIN_LIMIT_DEG = -180;   // set to protect cables
    public static double MAX_LIMIT_DEG = 180;

    static final double TICKS_PER_DEGREE =
            (TICKS_PER_MOTOR_REV * MOTOR_TURNS_PER_TURRET_TURN) / 360.0;

    @Override
    public void runOpMode() {
        DcMotor turret = hardwareMap.get(DcMotor.class, MOTOR_NAME);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double target = 0;
        double integral = 0;
        double lastAngle = 0;
        ElapsedTime timer = new ElapsedTime();

        telemetry.addLine("Right stick X = rotate | B = target 0 | Y = zero encoder here");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            double dt = timer.seconds();
            timer.reset();
            if (dt <= 0) continue;

            double angle = turret.getCurrentPosition() / TICKS_PER_DEGREE;

            // --- joystick moves the target ---
            double stick = gamepad1.right_stick_x;
            if (Math.abs(stick) < STICK_DEADZONE) stick = 0;
            target += stick * MAX_DEG_PER_SEC * dt;

            if (gamepad1.b) target = 0;
            if (gamepad1.y) { // re-zero: current position becomes 0
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                target = 0;
                angle = 0;
                integral = 0;
            }

            target = clamp(target, MIN_LIMIT_DEG, MAX_LIMIT_DEG);

            // --- PID ---
            double error = target - angle;

            if (Math.abs(error) < I_ZONE_DEG && kI > 0) {
                integral += error * dt;
                integral = clamp(integral, -I_MAX / kI, I_MAX / kI);
            } else {
                integral = 0;
            }

            // derivative on measurement: no kick when the target moves
            double derivative = -(angle - lastAngle) / dt;
            lastAngle = angle;

            double power = kP * error + kI * integral + kD * derivative;
            power = clamp(power, -MAX_POWER, MAX_POWER);

            if (Math.abs(error) <= TOLERANCE_DEG) {
                power = 0;
                integral = 0;
            }

            turret.setPower(power);

            telemetry.addData("Target", "%.1f", target);
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