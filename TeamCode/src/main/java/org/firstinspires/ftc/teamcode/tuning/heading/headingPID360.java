package org.firstinspires.ftc.teamcode.tuning.heading;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Heading Tuner Unwrapped (Pinpoint)", group = "Tuning")
@Config
public class headingPID360 extends OpMode {

    public static double nominalVoltage = 12.5;
    public static double kP = 0.55;
    public static double tanhScale = 2.0;
    public static double kS = 0.2;
    public static double maxPower = 1.0;
    public static double deadbandDeg = 0.4;
    public static double rightBias = 0.017;
    public static double targetHeadingDeg = 0.0;

    DcMotorEx stanga, dreapta;
    GoBildaPinpointDriver pinpoint;
    VoltageSensor voltageSensor;

    double unwrappedHeading = 0;
    double lastRawHeading = Double.NaN;

    @Override
    public void init() {
        stanga = hardwareMap.get(DcMotorEx.class, "stanga");
        dreapta = hardwareMap.get(DcMotorEx.class, "dreapta");

        stanga.setDirection(DcMotor.Direction.FORWARD);
        dreapta.setDirection(DcMotor.Direction.FORWARD);

        stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(50, 0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.resetPosAndIMU();

        unwrappedHeading = 0;
        lastRawHeading = Double.NaN;
    }

    @Override
    public void loop() {
        pinpoint.update();

        double rawHeading = pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
        if (!Double.isNaN(lastRawHeading)) {
            unwrappedHeading += normalizeAngle(rawHeading - lastRawHeading);
        }
        lastRawHeading = rawHeading;

        double error = targetHeadingDeg - unwrappedHeading;
        double batteryVoltage = voltageSensor.getVoltage();
        double scale = nominalVoltage / batteryVoltage;

        double power = 0;
        if (Math.abs(error) >= deadbandDeg) {
            double raw = Math.tanh(Math.toRadians(error) * tanhScale) * kP;
            power = (raw + Math.copySign(kS, raw)) * scale;
            power = Range.clip(power, -maxPower, maxPower);
        }

        double bias = power == 0 ? 0 : rightBias;
        stanga.setPower(power);
        dreapta.setPower(power + bias);

        telemetry.addData("Target", "%.1f°", targetHeadingDeg);
        telemetry.addData("Unwrapped", "%.1f°", unwrappedHeading);
        telemetry.addData("Raw heading", "%.1f°", rawHeading);
        telemetry.addData("Error", "%.2f°", error);
        telemetry.addData("Power", "%.3f", power);
        telemetry.addData("Battery V", "%.2f", batteryVoltage);
        telemetry.addData("Scale", "%.3f", scale);
        telemetry.update();
    }

    private double normalizeAngle(double angleDeg) {
        while (angleDeg > 180) angleDeg -= 360;
        while (angleDeg < -180) angleDeg += 360;
        return angleDeg;
    }
}