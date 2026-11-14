package org.firstinspires.ftc.teamcode.tuning.turela;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "tuning potentiometru", group = "tuning")
@Config
public class potentiometru extends OpMode {
    AnalogInput potTurela;
    @Override
    public void init() {
        potTurela = hardwareMap.get(AnalogInput.class, "potTurela");
    }


    @Override
    public void loop() {
        double v    = potTurela.getVoltage();     // volts, live reading
        double vMax = potTurela.getMaxVoltage();  // about 3.3 on REV hubs
        telemetry.addData("pot V", "%.3f", v);
        telemetry.addData("vMax", vMax);
        telemetry.addData("grade", potAngleDeg(v));
        telemetry.update();
    }

    double potAngleDeg(double v) {
        if (v < 1e-3) return 270.0;                 // avoid dividing by zero
        double b = 270.0 * v + 445.5;
        double c = 120285.0 - 36450.0 * v;
        return (b - Math.sqrt(b * b - 4.0 * v * c)) / (2.0 * v);
    }

}
