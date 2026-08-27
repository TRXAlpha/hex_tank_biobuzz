package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "test 120cm", group = "test")
public class test120 extends OpMode {
    DcMotor stanga, dreapta;
    GoBildaPinpointDriver pinpoint;
    int contor = 0;
    boolean mergeFata = true;
    boolean mergeSpate = false;

    @Override
    public void init() {
        stanga = hardwareMap.dcMotor.get("stanga");
        dreapta = hardwareMap.dcMotor.get("dreapta");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(-140, 60, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stanga.setDirection(DcMotor.Direction.REVERSE);
        dreapta.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {
        // Update sensor telemetry readings
        pinpoint.update();
        double currentX = pinpoint.getPosX(DistanceUnit.MM);

        if (contor < 10) {
            // Driving Forward to 1200 mm
            if (mergeFata) {
                if (currentX < 1200) {
                    stanga.setPower(0.5); // Using 0.5 power for better stopping precision
                    dreapta.setPower(0.5);
                } else {
                    stanga.setPower(0);
                    dreapta.setPower(0);
                    mergeFata = false;
                    mergeSpate = true;
                }
            }
            // Driving Backward to 0 mm
            else if (mergeSpate) {
                if (currentX > 0) {
                    stanga.setPower(-0.5);
                    dreapta.setPower(-0.5);
                } else {
                    stanga.setPower(0);
                    dreapta.setPower(0);
                    mergeSpate = false;
                    mergeFata = true;
                    contor++; // Completed one full forward-backward cycle
                }
            }
        } else {
            // Stop motors when 10 cycles are finished
            stanga.setPower(0);
            dreapta.setPower(0);
        }

        telemetry.addData("Cycles Completed", contor);
        telemetry.addData("X Position (mm)", currentX);
    }
}