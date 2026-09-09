package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="teleop debug", group="debugging")
public class arcadeTankDrive extends OpMode {
    private DcMotor stanga, dreapta;
    GoBildaPinpointDriver pinpoint;
    @Override
    public void init() {
        stanga  = hardwareMap.get(DcMotor.class, "stanga");
        dreapta = hardwareMap.get(DcMotor.class, "dreapta");
        stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Reverse the left motor so forward stick moves the robot forward
        stanga.setDirection(DcMotor.Direction.REVERSE);
        dreapta.setDirection(DcMotor.Direction.FORWARD);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        double podOffsetX_mm = 50;
        double podOffsetY_mm = 0;

        pinpoint.setOffsets(podOffsetX_mm, podOffsetY_mm, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
    }
    @Override
    public void start(){

    }
    @Override
    public void loop(){
        pinpoint.update();
        // Read joysticks (negated because pushing forward is negative by default)
        double putereStanga  = -gamepad1.left_stick_y;
        double putereDreapta = -gamepad1.right_stick_y;

        stanga.setPower(putereStanga);
        dreapta.setPower(putereDreapta);
        telemetry.addData("x", pinpoint.getPosX(DistanceUnit.CM));
        telemetry.addData("y", pinpoint.getPosY(DistanceUnit.CM));
        telemetry.addData("h", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("h (rad)", pinpoint.getHeading(AngleUnit.RADIANS));
        telemetry.addData("Motors", "Left (%.2f), Right (%.2f)", putereStanga, putereDreapta);
        telemetry.update();
    }
}

