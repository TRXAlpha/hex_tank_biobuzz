package org.firstinspires.ftc.teamcode.tuning.heading;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.tuning.fataspatetuning;

@TeleOp(name = "Heading Tuner Unwrapped (Pinpoint)", group = "Tuning")
@Config
public class headingPID360 extends OpMode {

    public static double nominalVoltage = 12.5;
    public static double kP = 0.55;
    public static double tanhScale = 2.0;
    public static double kS = 0.2;
    public static double kD = 0.002;
    public static double maxPower = 1.0;
    public static double deadbandDeg = 0.4;
    public static double rightBias = 0;
    public static double targetHeadingDeg = 0.0;

    // Pivot position relative to the Pinpoint point, in cm (minus = behind it). 0 = plain turn
    public static double corForwardCm = 0;
    public static double moveTolerance = 0.5;
    public static double settleRate = 15;

    private enum Phase { FORWARD, TURN, BACK, HOLD }

    DcMotorEx stanga, dreapta;
    GoBildaPinpointDriver pinpoint;
    VoltageSensor voltageSensor;

    double unwrappedHeading = 0;
    double lastRawHeading = Double.NaN;

    Phase phase = Phase.HOLD;
    double activeTarget = 0;
    double startX, startY, startHeading;

    @Override
    public void init() {
        // Sends telemetry to both the Driver Station and FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        stanga = hardwareMap.get(DcMotorEx.class, "stanga");
        dreapta = hardwareMap.get(DcMotorEx.class, "dreapta");

        stanga.setDirection(DcMotor.Direction.FORWARD);
        dreapta.setDirection(DcMotor.Direction.FORWARD);

        stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        // Measured from a 180 turn so the tracked point is the pivot. Needs a power restart to apply
        pinpoint.setOffsets(-44.3, -6.6, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
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

        double rate = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
        double x = pinpoint.getPosX(DistanceUnit.CM);
        double y = pinpoint.getPosY(DistanceUnit.CM);

        if (targetHeadingDeg != activeTarget) {
            activeTarget = targetHeadingDeg;
            startX = x;
            startY = y;
            startHeading = unwrappedHeading;
            phase = corForwardCm != 0 ? Phase.FORWARD : Phase.TURN;
        }

        double error = activeTarget - unwrappedHeading;
        double batteryVoltage = voltageSensor.getVoltage();
        double scale = nominalVoltage / batteryVoltage;

        double power = 0;
        if (Math.abs(error) >= deadbandDeg) {
            double raw = Math.tanh(Math.toRadians(error) * tanhScale) * kP;
            power = (raw + Math.copySign(kS, raw) - kD * rate) * scale;
            power = Range.clip(power, -maxPower, maxPower);
        }

        // A tank can't pivot around a point off its axle: put the pivot on the spot, turn, back off
        switch (phase) {
            case FORWARD:
                if (driveAlong(startHeading, -corForwardCm, x, y)) phase = Phase.TURN;
                break;
            case BACK:
                if (driveAlong(activeTarget, 0, x, y)) phase = Phase.HOLD;
                break;
            default:
                double bias = power == 0 ? 0 : rightBias;
                stanga.setPower(power);
                dreapta.setPower(power + bias);
                if (phase == Phase.TURN && Math.abs(error) < deadbandDeg && Math.abs(rate) < settleRate) {
                    phase = corForwardCm != 0 ? Phase.BACK : Phase.HOLD;
                }
        }

        // Raw numbers so Dashboard can graph them
        telemetry.addData("Target", targetHeadingDeg);
        telemetry.addData("Unwrapped", unwrappedHeading);
        telemetry.addData("Raw heading", rawHeading);
        telemetry.addData("Error", error);
        telemetry.addData("Rate", rate);
        telemetry.addData("Power", power);
        telemetry.addData("Battery V", batteryVoltage);
        telemetry.addData("Scale", scale);
        telemetry.addData("Phase", phase);
        telemetry.addData("Pinpoint X offset mm", pinpoint.getXOffset(DistanceUnit.MM));
        telemetry.addData("Pinpoint Y offset mm", pinpoint.getYOffset(DistanceUnit.MM));
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("Moved from start", Math.hypot(x - startX, y - startY));

        // After a plain turn, the distance moved tells us where the pivot is
        double a = Math.cos(Math.toRadians(startHeading)) - Math.cos(Math.toRadians(unwrappedHeading));
        double b = Math.sin(Math.toRadians(startHeading)) - Math.sin(Math.toRadians(unwrappedHeading));
        double chordSquared = a * a + b * b;
        if (corForwardCm == 0 && phase == Phase.HOLD && chordSquared >= 1) {
            telemetry.addData("Measured corForwardCm", (a * (x - startX) + b * (y - startY)) / chordSquared);
            telemetry.addData("Measured corLeftCm", (-b * (x - startX) + a * (y - startY)) / chordSquared);
        }
        telemetry.update();
    }

    private boolean driveAlong(double headingDeg, double goal, double x, double y) {
        double h = Math.toRadians(headingDeg);
        double along = (x - startX) * Math.cos(h) + (y - startY) * Math.sin(h);
        double error = goal - along;

        if (Math.abs(error) <= moveTolerance) {
            stanga.setPower(0);
            dreapta.setPower(0);
            return true;
        }

        double drive = Math.copySign(Range.clip(Math.abs(fataspatetuning.kPdrive * error),
                fataspatetuning.minDrive, fataspatetuning.maxDrive), error);
        double turn = Range.clip(fataspatetuning.kPturn * normalizeAngle(headingDeg - unwrappedHeading),
                -fataspatetuning.maxTurn, fataspatetuning.maxTurn);

        // Left motor is FORWARD here, so driving ahead needs negative power on it
        stanga.setPower(-(drive - turn));
        dreapta.setPower(drive + turn);
        return false;
    }

    private double normalizeAngle(double angleDeg) {
        while (angleDeg > 180) angleDeg -= 360;
        while (angleDeg < -180) angleDeg += 360;
        return angleDeg;
    }
}