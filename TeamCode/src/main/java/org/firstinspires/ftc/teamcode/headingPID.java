package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.internal.ftdi.eeprom.FT_EE_2232_Ctrl;

@TeleOp(name = "tuneHeading")
@Config
public class headingPID extends OpMode {

    FtcDashboard dashboard;
    public static double targetHeading = 90.0;

    public static double kP = 0.03;
    public static double kS = 0.2;

    public static double maxPower = 0.65;
    public static double deadband = 2.5;

    public static boolean invertTurn = false;

    private DcMotor stanga;
    private DcMotor dreapta;
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {

        stanga = hardwareMap.get(DcMotor.class, "stanga");
        dreapta = hardwareMap.get(DcMotor.class, "dreapta");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pinpoint.setOffsets(0, 55, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void start() {
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void loop() {
//
        pinpoint.update();
//        double heading = pinpoint.getHeading(AngleUnit.DEGREES);
//
//        // Difference between where we are and where we want to be
//        double error = wrap(targetHeading - heading);
//        double turn;
//        // Close enough -> stop
//        if (Math.abs(error) <= deadband) turn = 0.0;
//        else {
//            // P controller
//            turn = kP * error;
//            // Minimum power to overcome friction
//            if (Math.abs(turn) < kS) turn = Math.copySign(kS, turn);
//            // Limit maximum power
//            turn = clip(turn, -maxPower, maxPower);
//        }
//
//        if (invertTurn) turn = -turn;
//
//        // Turn CCW:
//        // left motor backwards
//        // right motor forwards
//        stanga.setPower(turn);
//        dreapta.setPower(turn);
//
//        telemetry.addData("Heading", "%.1f°", heading);
//        telemetry.addData("Target", "%.1f°", targetHeading);
//        telemetry.addData("Error", "%.1f°", error);
//        telemetry.addData("Turn Power", "%.3f", turn);

        telemetry.addData("heading", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData(
                "Position",
                "%.1f, %.1f cm",
                pinpoint.getPosX(DistanceUnit.CM),
                pinpoint.getPosY(DistanceUnit.CM)
        );

        telemetry.update();
    }

    private static double wrap(double degrees) {
        while (degrees >= 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }

    private static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}