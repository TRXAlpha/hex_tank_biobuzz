package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class outtake extends OpMode {
   DcMotor turelaD, turelaS;
    @Override
    public void init() {
        turelaD = hardwareMap.dcMotor.get("turelaD");
        turelaS = hardwareMap.dcMotor.get("turelaS");
        turelaD.setDirection(DcMotorSimple.Direction.FORWARD);
        turelaS.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        turelaD.setPower(gamepad1.left_stick_x);
        turelaS.setPower(gamepad1.left_stick_x);
        telemetry.addData("dreapta", gamepad1.left_stick_x);
        telemetry.addData("stanga", gamepad1.left_stick_x);
        telemetry.update();
    }
}
