package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@Autonomous(name = "Pinpoint Tank Navigator", group = "Autonomous")
public class PinpointTankNavigator extends OpMode {
    FtcDashboard dashboard;
    private DcMotor stanga, dreapta;
    private GoBildaPinpointDriver pinpoint;
    public static double maxPowerDrive = 0.6;
    public static double maxPowerTurn = 0.55;
    public static double kPturn = 0.03;
    public static double kDturn = 0.0015;
    public static double kPdrive = 0.018;
    public static double distIncet = 20;
    public static double tolerantaPos = 1;
    public static double targetX = 60.0;
    public static double thresholdHeading = 2;
    public static double targetY = 90.0;
    ElapsedTime turnTimer = new ElapsedTime();
    double previousError = 0.0;
    double previousTime = 0.0;

    @Override
    public void init() {
        stanga = hardwareMap.get(DcMotor.class, "stanga");
        dreapta = hardwareMap.get(DcMotor.class, "dreapta");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        dashboard = FtcDashboard.getInstance();
        stanga.setDirection(DcMotor.Direction.REVERSE);
        dreapta.setDirection(DcMotor.Direction.FORWARD);
        stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint.setOffsets(-140, 60, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();
        stopDrive();
    }

    @Override
    public void start() {
        turnTimer.reset();
        previousError = 0.0;
        previousTime = 0.0;

    }
    public static double distSwitchToFinalHeading = 5.0;
    public static double targetHeading = 0.0; // final desired heading, deg

    @Override
    public void loop() {
        pinpoint.update();

        double currentX = pinpoint.getPosX(DistanceUnit.CM);
        double currentY = pinpoint.getPosY(DistanceUnit.CM);
        double currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);

        double dx = targetX - currentX;
        double dy = targetY - currentY;
        double distance = Math.hypot(dx, dy);

        double travelHeading = Math.toDegrees(Math.atan2(dy, dx));
        double desiredHeading = (distance > distSwitchToFinalHeading) ? travelHeading : targetHeading;

        double headingError = angleWrap(desiredHeading - currentHeading);

        double currentTime = turnTimer.seconds();
        double dt = currentTime - previousTime;
        if (dt <= 0.0) dt = 0.001;

        double derivative = (headingError - previousError) / dt;
        double turnPower = kPturn * headingError + kDturn * derivative;
        turnPower = clip(turnPower, -maxPowerTurn, maxPowerTurn);
        if (abs(headingError) <= thresholdHeading) turnPower = 0;

        double drivePower = kPdrive * distance;
        if (distance > tolerantaPos) {
            drivePower = clip(drivePower, 0.12, maxPowerDrive);
            if (distance < distIncet) drivePower *= Math.max(0.25, distance / distIncet);
        } else drivePower = 0;

        if (abs(headingError) > 25.0) drivePower = 0.0;

        double leftPower = drivePower - turnPower;
        double rightPower = drivePower + turnPower;

        double maxPower = Math.max(1.0, Math.max(abs(leftPower), abs(rightPower)));
        leftPower /= maxPower;
        rightPower /= maxPower;

        stanga.setPower(leftPower);
        dreapta.setPower(rightPower);

        previousError = headingError;
        previousTime = currentTime;

        telemetry.addData("X", "%.1f cm", currentX);
        telemetry.addData("Y", "%.1f cm", currentY);
        telemetry.addData("Heading", "%.1f deg", currentHeading);
        telemetry.addData("Target", "%.1f, %.1f, hdg %.1f", targetX, targetY, targetHeading);
        telemetry.addData("Distance", "%.1f cm", distance);
        telemetry.addData("Hdg Err", "%.1f deg", headingError);
        telemetry.update();
    }
    private double angleWrap(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void stopDrive() {
        stanga.setPower(0.0);
        dreapta.setPower(0.0);
    }
}