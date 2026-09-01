package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/**
 * Straight-line drive calibration.
 *
 * <p>This isolates the longitudinal controller: it drives distanceCm along
 * the heading recorded at START and uses the heading controller only to keep
 * the robot straight. Tune drive kP/kD/kS first, then tune the heading values
 * in {@link headingPID}.</p>
 */
@TeleOp(name = "tuneDrive")
@Config
public class drivePID extends OpMode {

    public static double distanceCm = 50.0;
    public static double kP = 0.018;
    public static double kD = 0.004;
    public static double kS = 0.08;
    public static double headingkP = 0.030;
    public static double headingkD = 0.003;
    public static double headingkS = 0.10;
    public static double maxPower = 0.45;
    public static double maxTurn = 0.15;
    public static double distanceToleranceCm = 2.0;
    public static double headingToleranceDeg = 1.5;
    public static boolean invertDrive = false;
    public static boolean invertTurn = false;

    private DcMotor stanga;
    private DcMotor dreapta;
    private GoBildaPinpointDriver pinpoint;
    private double startH;
    private boolean enabled;

    @Override
    public void init() {
        stanga = hardwareMap.get(DcMotor.class, "stanga");
        dreapta = hardwareMap.get(DcMotor.class, "dreapta");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");


        stanga.setDirection(DcMotor.Direction.REVERSE);
        dreapta.setDirection(DcMotor.Direction.FORWARD);
        stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pinpoint.setOffsets(-140.0, 60.0, DistanceUnit.MM);
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
        pinpoint.resetPosAndIMU();
        pinpoint.update();
        startH = pinpoint.getHeading(AngleUnit.DEGREES);
        enabled = true;
    }

    @Override
    public void loop() {
        if (gamepad1.b) enabled = false;
        if (!enabled) {
            stopDrive();
            telemetry.addLine("STOPPED - restart the OpMode to run again");
            telemetry.update();
            return;
        }

        pinpoint.update();
        double heading = pinpoint.getHeading(AngleUnit.DEGREES);
        double headingVelocity = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

        // Project field displacement and velocity onto the starting forward
        // axis. This makes the test remain valid even if the robot starts at
        // a non-zero field heading.
        double startRadians = Math.toRadians(startH);
        double x = pinpoint.getPosX(DistanceUnit.CM);
        double y = pinpoint.getPosY(DistanceUnit.CM);
        double distanceAlongStartHeading = x * Math.cos(startRadians)
                + y * Math.sin(startRadians);
        double velocityAlongStartHeading = pinpoint.getVelX(DistanceUnit.CM) * Math.cos(startRadians)
                + pinpoint.getVelY(DistanceUnit.CM) * Math.sin(startRadians);

        double driveError = distanceCm - distanceAlongStartHeading;
        double headingError = wrap(startH - heading);

        double drive = Math.abs(driveError) <= distanceToleranceCm
                ? 0.0
                : command(driveError, velocityAlongStartHeading,
                kP, kD, kS, maxPower);
        double turn = Math.abs(headingError) <= headingToleranceDeg
                ? 0.0
                : command(headingError, headingVelocity,
                headingkP, headingkD, headingkS, maxTurn);

        if (invertDrive) drive = -drive;
        if (invertTurn) turn = -turn;
        setTankPower(drive - turn, drive + turn);

        telemetry.addData("Start heading", "%.1f deg", startH);
        telemetry.addData("Distance", "%.1f / %.1f cm", distanceAlongStartHeading, distanceCm);
        telemetry.addData("Drive error / power", "%.1f / %.3f", driveError, drive);
        telemetry.addData("Heading / error", "%.1f / %.1f deg", heading, headingError);
        telemetry.addData("Turn power", "%.3f", turn);
        telemetry.addData("Lateral drift", "%.1f cm", -x * Math.sin(startRadians)
                + y * Math.cos(startRadians));
        telemetry.update();
    }

    @Override
    public void stop() {
        stopDrive();
    }

    private void setTankPower(double left, double right) {
        double peak = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)));
        stanga.setPower(left / peak);
        dreapta.setPower(right / peak);
    }

    private void stopDrive() {
        if (stanga != null) stanga.setPower(0.0);
        if (dreapta != null) dreapta.setPower(0.0);
    }

    private static double command(double error, double velocity,
                                  double kP, double kD, double kS, double max) {
        max = clip(max, 0.0, 1.0);
        kS = clip(kS, 0.0, max);
        double raw = Math.tanh(kP * error - kD * velocity);
        if (Math.abs(raw) < 1e-5 || max == 0.0) return 0.0;
        return Math.copySign(kS + (max - kS) * Math.abs(raw), raw);
    }

    private static double wrap(double degrees) {
        while (degrees >= 180.0) degrees -= 360.0;
        while (degrees < -180.0) degrees += 360.0;
        return degrees;
    }

    private static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
