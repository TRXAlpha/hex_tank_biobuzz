package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp (name = "test auto cu coordonate")
@Disabled
public class auto extends OpMode {
    GoBildaPinpointDriver pinpoint;
    DcMotor stanga, dreapta;

    @Override
    public void init() {
        stanga = hardwareMap.dcMotor.get("stanga");
        dreapta = hardwareMap.dcMotor.get("dreapta");
        stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // pp
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-140, 60, DistanceUnit.MM);
    }

    @Override
    public void loop() {

    }
}
