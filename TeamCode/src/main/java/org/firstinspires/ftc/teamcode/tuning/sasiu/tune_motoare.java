package org.firstinspires.ftc.teamcode.tuning.sasiu;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp(name = "tune motoare", group = "tuning")
//@Config
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
