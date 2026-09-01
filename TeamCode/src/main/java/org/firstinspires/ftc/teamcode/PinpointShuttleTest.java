package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@Autonomous(name = "Pinpoint Shuttle Test", group = "Test")
public class PinpointShuttleTest extends OpMode {

    public static double legDistance = 100.0;
    public static int cycles = 10;

    public static double kPdrive = 0.022;
    public static double minDrive = 0.14;
    public static double maxDrive = 0.55;

    public static double kPturn = 0.025;
    public static double kDturn = 0.0015;
    public static double maxTurn = 0.35;

    public static double kCross = 1.5;
    public static double maxCross = 20.0;

    public static double posTolerance = 1.5;
    public static double settleTime = 0.25;
    public static double legTimeout = 6.0;

    private DcMotor stanga, dreapta;
    private GoBildaPinpointDriver pinpoint;

    private final ElapsedTime legTimer = new ElapsedTime();
    private final ElapsedTime settleTimer = new ElapsedTime();

    private int leg = 0;
    private double targetX = 0.0;
    private boolean finished = false;
    private boolean firstLoop = true;
    private double prevHeading = 0.0;
    private double prevTime = 0.0;

    @Override
    public void init() {
        stanga = hardwareMap.get(DcMotor.class, "stanga");
        dreapta = hardwareMap.get(DcMotor.class, "dreapta");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

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
        setPower(0, 0);
    }

    @Override
    public void init_loop() {
        pinpoint.update();
        telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
        telemetry.addData("Pose", "%.1f, %.1f, %.1f",
                pinpoint.getPosX(DistanceUnit.CM),
                pinpoint.getPosY(DistanceUnit.CM),
                pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addLine("Push the robot by hand and confirm X/Y/heading move the right way.");
        telemetry.update();
    }

    @Override
    public void start() {
        leg = 0;
        targetX = legDistance;
        finished = false;
        firstLoop = true;
        prevHeading = 0.0;
        prevTime = 0.0;
        legTimer.reset();
        settleTimer.reset();
    }

    @Override
    public void loop() {
        pinpoint.update();

        double x = pinpoint.getPosX(DistanceUnit.CM);
        double y = pinpoint.getPosY(DistanceUnit.CM);
        double heading = pinpoint.getHeading(AngleUnit.DEGREES);

        if (finished) {
            setPower(0, 0);
            telemetry.addLine("Done");
            telemetry.addData("Residual", "%.2f, %.2f, %.2f deg", x, y, heading);
            telemetry.update();
            return;
        }

        double error = targetX - x;
        double direction = Math.signum(error);

        double now = legTimer.seconds();
        double dt = now - prevTime;
        if (dt < 1e-4) dt = 1e-4;

        double headingRate = firstLoop ? 0.0 : (heading - prevHeading) / dt;

        double desiredHeading = Range.clip(-kCross * y, -maxCross, maxCross) * direction;
        double headingError = angleWrap(desiredHeading - heading);

        double turn = Range.clip(kPturn * headingError - kDturn * headingRate, -maxTurn, maxTurn);

        double drive;
        if (Math.abs(error) <= posTolerance) {
            drive = 0.0;
        } else {
            drive = direction * Range.clip(Math.abs(kPdrive * error), minDrive, maxDrive);
            settleTimer.reset();
        }

        double left = drive - turn;
        double right = drive + turn;
        double norm = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)));
        setPower(left / norm, right / norm);

        prevHeading = heading;
        prevTime = now;
        firstLoop = false;

        telemetry.addData("Leg", "%d / %d  (%s)", leg + 1, cycles * 2, leg % 2 == 0 ? "out" : "back");
        telemetry.addData("X / target", "%.1f / %.1f cm", x, targetX);
        telemetry.addData("Cross track Y", "%.2f cm", y);
        telemetry.addData("Heading", "%.1f deg (err %.1f)", heading, headingError);
        telemetry.addData("Drive / Turn", "%.2f / %.2f", drive, turn);
        telemetry.addData("Loop", "%.0f Hz", 1.0 / dt);
        telemetry.update();

        boolean settled = Math.abs(error) <= posTolerance && settleTimer.seconds() >= settleTime;
        if (settled || legTimer.seconds() >= legTimeout) nextLeg();
    }

    @Override
    public void stop() {
        setPower(0, 0);
    }

    private void nextLeg() {
        setPower(0, 0);
        leg++;
        if (leg >= cycles * 2) {
            finished = true;
            return;
        }
        targetX = (leg % 2 == 0) ? legDistance : 0.0;
        firstLoop = true;
        prevTime = 0.0;
        legTimer.reset();
        settleTimer.reset();
    }

    private void setPower(double left, double right) {
        stanga.setPower(left);
        dreapta.setPower(right);
    }

    private double angleWrap(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
}