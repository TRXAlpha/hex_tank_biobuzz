package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "Square Drive Pinpoint", group = "Auto")
public class gtptestguzu extends LinearOpMode {

    private DcMotor leftDrive  = null;
    private DcMotor rightDrive = null;
    private GoBildaPinpointDriver pinpoint = null;

    private double headingError = 0;
    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed  = 0;
    private double leftSpeed  = 0;
    private double rightSpeed = 0;
    private int leftTarget  = 0;
    private int rightTarget = 0;

    // ===== Tunable constants =====
    static final double COUNTS_PER_MOTOR_REV = 537.7;   // GoBILDA 312 RPM
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double DRIVE_SPEED = 0.85;
    static final double TURN_SPEED  = 0.70;

    static final double HEADING_THRESHOLD = 1.5;
    static final double P_TURN_GAIN  = 0.03;
    static final double P_DRIVE_GAIN = 0.04;

    static final double SIDE_LENGTH = 10.6;   // ← 600 mm

    @Override
    public void runOpMode() {

        // Motors – your config names
        leftDrive  = hardwareMap.get(DcMotor.class, "stanga");
        rightDrive = hardwareMap.get(DcMotor.class, "dreapta");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();

        // Wait for start – show live heading
        while (opModeInInit()) {
            pinpoint.update();
            telemetry.addData("Heading", "%.1f°", getHeading());
            telemetry.addData("Status", "Ready – press START");
            telemetry.update();
        }

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reset position & heading at start
        pinpoint.resetPosAndIMU();
        sleep(200);

        // ========== THE SQUARE ==========
        for (int i = 0; i < 4; i++) {
            double currentHeading = -90.0 * i;

            driveStraight(DRIVE_SPEED, SIDE_LENGTH, currentHeading);
            turnToHeading(TURN_SPEED, currentHeading - 90.0);
            holdHeading(TURN_SPEED, currentHeading - 90.0, 0.15);
        }

        telemetry.addData("Path", "Square complete");
        telemetry.update();
        sleep(1000);
    }

    // -------------------- Pinpoint setup --------------------
    private void configurePinpoint() {
        pinpoint.setOffsets(55.0, 0.0, DistanceUnit.MM);

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();
    }

    // -------------------- High-level helpers --------------------
    public void driveStraight(double maxDriveSpeed, double distance, double heading) {
        if (!opModeIsActive()) return;

        int moveCounts = (int) (distance * COUNTS_PER_INCH);
        leftTarget  = leftDrive.getCurrentPosition()  + moveCounts;
        rightTarget = rightDrive.getCurrentPosition() + moveCounts;

        leftDrive.setTargetPosition(leftTarget);
        rightDrive.setTargetPosition(rightTarget);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        maxDriveSpeed = Math.abs(maxDriveSpeed);
        moveRobot(maxDriveSpeed, 0);

        while (opModeIsActive() && leftDrive.isBusy() && rightDrive.isBusy()) {
            pinpoint.update();
            turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
            if (distance < 0) turnSpeed *= -1.0;
            moveRobot(driveSpeed, turnSpeed);
            sendTelemetry(true);
        }

        moveRobot(0, 0);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnToHeading(double maxTurnSpeed, double heading) {
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        while (opModeIsActive() && Math.abs(headingError) > HEADING_THRESHOLD) {
            pinpoint.update();
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            moveRobot(0, turnSpeed);
            sendTelemetry(false);
        }
        moveRobot(0, 0);
    }

    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (opModeIsActive() && holdTimer.seconds() < holdTime) {
            pinpoint.update();
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            moveRobot(0, turnSpeed);
            sendTelemetry(false);
        }
        moveRobot(0, 0);
    }

    // -------------------- Low-level helpers --------------------
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;
        headingError  = targetHeading - getHeading();

        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return Range.clip(headingError * proportionalGain, -1.0, 1.0);
    }

    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;
        turnSpeed  = turn;

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed  /= max;
            rightSpeed /= max;
        }

        leftDrive.setPower(leftSpeed);
        rightDrive.setPower(rightSpeed);
    }

    private void sendTelemetry(boolean straight) {
        Pose2D pose = pinpoint.getPosition();

        if (straight) {
            telemetry.addData("Motion", "Straight");
            telemetry.addData("Target L:R", "%7d : %7d", leftTarget, rightTarget);
            telemetry.addData("Actual L:R", "%7d : %7d",
                    leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading Target:Current", "%.1f : %.1f", targetHeading, getHeading());
        telemetry.addData("Error : Steer", "%.1f : %.2f", headingError, turnSpeed);
        telemetry.addData("Pose X:Y", "%.1f : %.1f",
                pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH));
        telemetry.addData("Powers L:R", "%.2f : %.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    public double getHeading() {
        return pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
    }
}