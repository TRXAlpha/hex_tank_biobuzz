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
@Autonomous(name = "Smooth Shuttle (Clean + PD)", group = "Test")
public class drivetanh extends OpMode {

    // ========== Tunable ==========
    public static double legDistance   = 100.0;   // cm
    public static int    maxCycles     = 3;

    public static double kPdrive       = 0.06;
    public static double kD            = 0.0075;   // ← start here, raise if still overshoots
    public static double kS            = 0.2;    // static friction compensation
    public static double maxDrive      = 1;

    public static double kPturn        = 0.1;
    public static double maxTurn       = 0.30;
    public static double kCross        = 1.5;

    public static double posTolerance  = 0.2;     // cm
    public static double velTolerance  = 5.0;     // cm/s  (used for better settle)
    public static double settleTime    = 0.20;    // seconds
    public static double legTimeout    = 6.0;

    // ========== Hardware ==========
    private DcMotor stanga, dreapta;
    private GoBildaPinpointDriver pinpoint;

    // ========== State ==========
    private final ElapsedTime legTimer    = new ElapsedTime();
    private final ElapsedTime settleTimer = new ElapsedTime();

    private double currentTarget = 0;
    private boolean goingToTarget = true;
    private int completedCycles = 0;
    private boolean finished = false;

    @Override
    public void init() {
        stanga  = hardwareMap.get(DcMotor.class, "stanga");
        dreapta = hardwareMap.get(DcMotor.class, "dreapta");

        stanga.setDirection(DcMotor.Direction.REVERSE);
        dreapta.setDirection(DcMotor.Direction.FORWARD);
        stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-140, 60, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();

        currentTarget   = legDistance;
        goingToTarget   = true;
        completedCycles = 0;
        finished        = false;
    }

    @Override
    public void start() {
        legTimer.reset();
        settleTimer.reset();
    }

    @Override
    public void loop() {
        pinpoint.update();

        double x         = pinpoint.getPosX(DistanceUnit.CM);
        double y         = pinpoint.getPosY(DistanceUnit.CM);
        double heading   = pinpoint.getHeading(AngleUnit.DEGREES);
        double velocityX = pinpoint.getVelX(DistanceUnit.CM);   // cm/s

        if (finished) {
            setPower(0, 0);
            return;
        }

        // ---------- Error & direction ----------
        double error     = currentTarget - x;
        double direction = Math.signum(error);

        // ---------- Heading (cross-track) ----------
        double desiredHeading = Range.clip(-kCross * y, -20, 20) * direction;
        double headingError   = angleWrap(desiredHeading - heading);
        double turn           = Range.clip(kPturn * headingError, -maxTurn, maxTurn);

        // ---------- Drive (P + D + kS) ----------
        double drive = 0;

        if (Math.abs(error) > posTolerance) {
            double pTerm = kPdrive * error;
            double dTerm = -kD * velocityX;
            double raw   = pTerm + dTerm;

            // static friction compensation
            raw += Math.copySign(kS, raw);

            drive = Range.clip(raw, -maxDrive, maxDrive);
            settleTimer.reset();
        }

        // Apply powers
        double left  = drive - turn;
        double right = drive + turn;
        double max   = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)));
        setPower(left / max, right / max);

        // ---------- Arrival logic ----------
        boolean settled = Math.abs(error) <= posTolerance
                && Math.abs(velocityX) < velTolerance
                && settleTimer.seconds() >= settleTime;

        boolean timedOut = legTimer.seconds() >= legTimeout;

        if (settled || timedOut) {
            setPower(0, 0);

            if (goingToTarget) {
                currentTarget = 0;
                goingToTarget = false;
            } else {
                completedCycles++;
                if (completedCycles >= maxCycles) {
                    finished = true;
                } else {
                    currentTarget = legDistance;
                    goingToTarget = true;
                }
            }

            legTimer.reset();
            settleTimer.reset();
        }

        // ---------- Telemetry ----------
        telemetry.addData("Target", "%.0f cm", currentTarget);
        telemetry.addData("X / Error", "%.1f / %.1f cm", x, error);
        telemetry.addData("Velocity X", "%.1f cm/s", velocityX);
        telemetry.addData("Y drift", "%.2f cm", y);
        telemetry.addData("Heading err", "%.1f°", headingError);
        telemetry.addData("Drive / Turn", "%.2f / %.2f", drive, turn);
        telemetry.addData("Going to target?", goingToTarget);
        telemetry.addData("Cycles", "%d / %d", completedCycles, maxCycles);
        telemetry.addData("Settled?", settled);
        telemetry.addData("--- Gains ---", "");
        telemetry.addData("kP", kPdrive);
        telemetry.addData("kD", kD);
        telemetry.addData("kS", kS);
        telemetry.update();
    }

    // ---------- Helpers ----------
    private void setPower(double left, double right) {
        stanga.setPower(left);
        dreapta.setPower(right + 0.017);
    }

    private double angleWrap(double angle) {
        while (angle >  180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}