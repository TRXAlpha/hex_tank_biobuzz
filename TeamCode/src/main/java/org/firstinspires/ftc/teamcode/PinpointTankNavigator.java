package org.firstinspires.ftc.teamcode;

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
    private DcMotor stanga;
    private DcMotor dreapta;
    private GoBildaPinpointDriver pinpoint;

    public static double MAX_DRIVE_POWER = 0.6;
    public static double MAX_TURN_POWER = 0.55;
    public static double TURN_KP = 0.03;
    public static double TURN_KD = 0.0015;
    public static double DRIVE_KP = 0.018;
    public static double SLOW_DISTANCE_CM = 20;
    public static double POSITION_TOLERANCE_CM = 1;
    public static double targetX = 60.0;
    public static double targetY = 90.0;
    private boolean finished = false;

    private final ElapsedTime turnTimer = new ElapsedTime();
    private double previousError = 0.0;
    private double previousTime = 0.0;

    @Override
    public void init() {
        stanga = hardwareMap.get(DcMotor.class, "stanga");
        dreapta = hardwareMap.get(DcMotor.class, "dreapta");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        dashboard = FtcDashboard.getInstance();
        stanga.setDirection(DcMotor.Direction.FORWARD);
        dreapta.setDirection(DcMotor.Direction.REVERSE);
        stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint.setOffsets(-140, 60, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.resetPosAndIMU();
        stopDrive();
    }

    @Override
    public void start() {
        turnTimer.reset();
        previousError = 0.0;
        previousTime = 0.0;
        finished = false;
    }

    @Override
    public void loop() {
        if (finished) stopDrive();

        pinpoint.update();

        double currentX = pinpoint.getPosX(DistanceUnit.MM);
        double currentY = pinpoint.getPosY(DistanceUnit.MM);
        double currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);

        double dx = targetX - currentX;
        double dy = targetY - currentY;
        double distance = Math.hypot(dx, dy);

        if (distance <= POSITION_TOLERANCE_CM) {
            stopDrive();
            finished = true;
        }

        double targetHeading = Math.toDegrees(Math.atan2(dy, dx));
        double headingError = angleWrap(targetHeading - currentHeading);

        // derivata
        double currentTime = turnTimer.seconds();
        double dt = currentTime - previousTime;
        if (dt <= 0.0) dt = 0.001;

        double derivative = (headingError - previousError) / dt;
        double turnPower = TURN_KP * headingError + TURN_KD * derivative;
        turnPower = clip(turnPower, -MAX_TURN_POWER, MAX_TURN_POWER);

        double drivePower = DRIVE_KP * distance;
        drivePower = clip(drivePower, 0.12, MAX_DRIVE_POWER);

        if (distance < SLOW_DISTANCE_CM) drivePower *= Math.max(0.25, distance / SLOW_DISTANCE_CM);

        if (Math.abs(headingError) > 25.0) drivePower = 0.0;


        double leftPower = drivePower + turnPower;
        double rightPower = drivePower - turnPower;

        double maxPower = Math.max(1.0, Math.max(Math.abs(leftPower), Math.abs(rightPower)));
        leftPower /= maxPower;
        rightPower /= maxPower;

        stanga.setPower(leftPower);
        dreapta.setPower(rightPower);

        previousError = headingError;
        previousTime = currentTime;

        telemetry.addData("X", "%.1f cm", currentX);
        telemetry.addData("Y", "%.1f cm", currentY);
        telemetry.addData("Heading", "%.1f deg", currentHeading);
        telemetry.addData("Target", "%.1f, %.1f", targetX, targetY);
        telemetry.addData("Distance", "%.1f cm", distance);
        telemetry.addData("Hdg Err", "%.1f deg", headingError);
        telemetry.update();
    }

    @Override
    public void stop() {
        stopDrive();
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