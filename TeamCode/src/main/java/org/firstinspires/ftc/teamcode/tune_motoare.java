package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Straight-line drive calibration.
 *
 * <p>This isolates the longitudinal controller: it drives distanceCm along
 * the heading recorded at START and uses the heading controller only to keep
 * the robot straight. Tune drive kP/kD/kS first, then tune the heading values
 * in {@link headingPID}.</p>
 */
@TeleOp(name = "tune motoare")
@Config
public class tune_motoare extends OpMode {
    private DcMotor stanga;
    private DcMotor dreapta;
    public static double power = 0.1;
    public static double factorD = 0.02;
    @Override
    public void init() {
        stanga = hardwareMap.get(DcMotor.class, "stanga");
        dreapta = hardwareMap.get(DcMotor.class, "dreapta");
        stanga.setDirection(DcMotor.Direction.REVERSE);
        dreapta.setDirection(DcMotor.Direction.FORWARD);
        stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {
        stanga.setPower(power);
        dreapta.setPower(power + factorD);
    }

}
