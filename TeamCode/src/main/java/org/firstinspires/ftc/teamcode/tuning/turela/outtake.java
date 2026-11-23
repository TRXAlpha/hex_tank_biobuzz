package org.firstinspires.ftc.teamcode.tuning.turela;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(group = "tuning")
public class outtake extends OpMode {
    public static double viteza = 0.7;
    public static double VITEZA_STAGIU = 1, VITEZA_INTAKE = 1;
   DcMotor turelaD, turelaS, stagiu, intake;
    @Override
    public void init() {
        intake = hardwareMap.dcMotor.get("intake");
        stagiu = hardwareMap.dcMotor.get("stagiu");
        stagiu.setDirection(DcMotorSimple.Direction.REVERSE);
        turelaD = hardwareMap.dcMotor.get("turelaD");
        turelaS = hardwareMap.dcMotor.get("turelaS");
        turelaD.setDirection(DcMotorSimple.Direction.FORWARD);
        turelaS.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        intake.setPower(VITEZA_INTAKE);
        stagiu.setPower(VITEZA_STAGIU);
        turelaD.setPower(viteza);
        turelaS.setPower(viteza);

    }
}
