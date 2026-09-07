package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name="teleop driving", group = "tank")
public class teleopDriving extends OpMode {
    private DcMotor stanga,dreapta;
    @Override
    public void init() {
        stanga = hardwareMap.get(DcMotor.class, "stanga");
        dreapta = hardwareMap.get(DcMotor.class, "dreapta");
        stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        //bleh
        double forward = -gamepad1.right_stick_x;
        double turn = gamepad1.left_stick_y;

        double putereStanga = forward+turn;
        double putereDreapta = forward-turn;
        stanga.setPower(putereStanga);
        dreapta.setPower(putereDreapta);

        telemetry.addData("Motors", "Left (%.2f), Right (%.2f)", putereStanga, putereDreapta);
        telemetry.update();
    }
}
