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
@Autonomous(name = "fata spate fata spate")
public class TankShuttleTest extends OpMode {

    public static double legDistance = 100;
    public static int cycles = 3;

    public static double kPdrive = 0.0070;
    public static double minDrive = 0.15;
    public static double maxDrive = 1;

    public static double kPturn = 0.04;
    public static double maxTurn = 0.3;

    public static double kCross = 1.5;

    public static double kDdrive = 0.003;

    public static double posTolerance = 1.5;
    public static double settleTime = 0.25;
    public static double legTimeout = 6.0;

    private static final double MAX_CROSS_DEG = 20.0;

    private DcMotor stanga, dreapta;
    private GoBildaPinpointDriver pinpoint;

    private final ElapsedTime legTimer = new ElapsedTime();
    private final ElapsedTime settleTimer = new ElapsedTime();

    private int distanta;
    private double targetX;
    private boolean finished;
    private double prevTime;

    private double previousError = 0;

    @Override
    public void init() {
        stanga = hardwareMap.get(DcMotor.class, "stanga");
        dreapta = hardwareMap.get(DcMotor.class, "dreapta");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        stanga.setDirection(DcMotor.Direction.REVERSE);
        dreapta.setDirection(DcMotor.Direction.FORWARD);
        stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint.setOffsets(0, 55, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        pinpoint.resetPosAndIMU();
        setPower(0, 0);
    }

    @Override
    public void start() {
        distanta = 0;
        targetX = legDistance;
        finished = false;
        prevTime = 0.0;
        legTimer.reset();
        settleTimer.reset();
        previousError = 0;
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
            telemetry.addData("Residual", "%.2f cm, %.2f cm, %.2f deg", x, y, heading);
            telemetry.update();
            return;
        }

        double error = targetX - x;
        double direction = Math.signum(error);

        double now = legTimer.seconds();

        double dt = now - prevTime;

        double desiredHeading = Range.clip(-kCross * y, -MAX_CROSS_DEG, MAX_CROSS_DEG) * direction;
        double headingError = angleWrap(desiredHeading - heading);
        double turn = Range.clip(kPturn * headingError, -maxTurn, maxTurn);
        double D = 0;

        if (dt > 0) {
            D = (error - previousError) / dt;
        }

        double drive = 0.0;
        if (Math.abs(error) > posTolerance) {
            drive = direction * Range.clip(Math.abs(kPdrive * error + kDdrive * D), minDrive, maxDrive);
            settleTimer.reset();
        }

        double left = drive - turn;
        double right = drive + turn;
        setPower(left , right);

        prevTime = now;
        previousError = error;

        telemetry.addData("Leg", "%d / %d  %s", distanta + 1, cycles * 2, distanta % 2 == 0 ? "out" : "back");
        telemetry.addData("X / target", "%.1f / %.1f cm", x, targetX);
        telemetry.addData("Cross track Y", "%.2f cm", y);
        telemetry.addData("Heading", "%.1f deg (err %.1f)", heading, headingError);
        telemetry.addData("Drive / Turn", "%.2f / %.2f", drive, turn);
        telemetry.addData("Loop", "%.0f Hz", 1.0 / dt);
        telemetry.update();

        boolean settled = Math.abs(error) <= posTolerance && settleTimer.seconds() >= settleTime;
        if (settled || legTimer.seconds() >= legTimeout) nextLeg();
    }

    private void nextLeg() {
        setPower(0, 0);
        distanta++;
        if (distanta >= cycles * 2) {
            finished = true;
            return;
        }
        targetX = (distanta % 2 == 0) ? legDistance : 0.0;
        prevTime = 0.0;
        previousError = 0;
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