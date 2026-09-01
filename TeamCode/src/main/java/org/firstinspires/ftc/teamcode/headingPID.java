package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Smooth Heading Drive Tuner (Pinpoint)", group = "Tuning")
@Config
public class headingPID extends OpMode {

    public static double nominalVoltage = 12.5;
    public static double kP = 0.55;
    public static double tanhScale = 2.0;
    public static double kS = 0.2;
    public static double maxPower = 1.0;
    public static double targetHeadingDeg = 0.0;

    DcMotorEx stanga, dreapta;
    GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        stanga = hardwareMap.get(DcMotorEx.class, "stanga");
        dreapta = hardwareMap.get(DcMotorEx.class, "dreapta");

        stanga.setDirection(DcMotor.Direction.FORWARD);
        dreapta.setDirection(DcMotor.Direction.FORWARD);

        stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(0, 55, DistanceUnit.MM); // change these to your real offsets
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void loop() {
        pinpoint.update();

        double batteryVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        Pose2D pose = pinpoint.getPosition();
        double currentHeading = pose.getHeading(AngleUnit.DEGREES);

        double error = targetHeadingDeg - currentHeading;

        // tanh controller
        double raw = Math.tanh(Math.toRadians(error) * tanhScale) * kP;

        double power;
        if (Math.abs(error) < 0.4) power = 0;
        else power = raw + Math.copySign(kS, raw);


        // Voltage compensation
        double scale = nominalVoltage / batteryVoltage;
        double compensatedPower = power * scale;

        // Optional: boost kS a bit when battery is low
        double dynamicKS = kS * (12.0 / batteryVoltage);
        if (Math.abs(error) >= 0.4) {
            compensatedPower += Math.copySign(dynamicKS - kS, compensatedPower);
        }

        compensatedPower = Range.clip(compensatedPower, -maxPower, maxPower);

        stanga.setPower(compensatedPower);
        dreapta.setPower(compensatedPower + 0.017);

        // Telemetry
        telemetry.addData("Target", "%.1f°", targetHeadingDeg);
        telemetry.addData("Current", "%.1f°", currentHeading);
        telemetry.addData("Error", "%.2f°", error);
        telemetry.addData("Raw Power", "%.3f", power);
        telemetry.addData("Compensated", "%.3f", compensatedPower);
        telemetry.addData("Battery V", "%.2f", batteryVoltage);
        telemetry.addData("Scale", "%.3f", scale);
        telemetry.addData("kP", kP);
        telemetry.addData("tanhScale", tanhScale);
        telemetry.addData("kS", kS);
        telemetry.update();
    }
}