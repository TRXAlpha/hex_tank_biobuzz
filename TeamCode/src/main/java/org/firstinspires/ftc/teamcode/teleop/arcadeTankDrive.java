package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="teleop debug", group="debugging")
public class arcadeTankDrive extends OpMode {
    private DcMotor stanga, dreapta;
    @Override
    public void init() {
        stanga  = hardwareMap.get(DcMotor.class, "stanga");
        dreapta = hardwareMap.get(DcMotor.class, "dreapta");
        stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Reverse the left motor so forward stick moves the robot forward
        stanga.setDirection(DcMotor.Direction.REVERSE);
        dreapta.setDirection(DcMotor.Direction.FORWARD);
    }
    @Override
    public void start(){

    }
    @Override
    public void loop(){

        // Read joysticks (negated because pushing forward is negative by default)
        double putereStanga  = -gamepad1.left_stick_y;
        double putereDreapta = -gamepad1.right_stick_y;

        stanga.setPower(putereStanga);
        dreapta.setPower(putereDreapta);

        telemetry.addData("Motors", "Left (%.2f), Right (%.2f)", putereStanga, putereDreapta);
        telemetry.update();
    }
}

