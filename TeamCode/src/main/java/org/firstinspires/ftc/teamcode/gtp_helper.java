package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class gtp_helper {

    public static double DISTANCE_TOLERANCE_MM = 10;
    public static double HEADING_LOCK_DEG = 10;
    public static double HEADING_DEADBAND_DEG = 1.5;
    public static double KP_DRIVE = 0.0035;
    public static double KP_TURN = 0.02;
    public static double MAX_POWER = 0.75;
    public static double MIN_DRIVE_POWER = 0.25;
    public static double MIN_TURN_POWER = 0.25;
    public static double TRANSLATE_TURN_SCALE = 0.5;

    public static double STALL_TIMEOUT_S = 1.0;
    public static double STALL_DISTANCE_MM = 2.0;
    public static double STALL_HEADING_DEG = 0.5;

    private final DcMotor leftDrive;
    private final DcMotor rightDrive;
    private final GoBildaPinpointDriver pinpoint;

    private final ElapsedTime stallTimer = new ElapsedTime();
    private boolean stallTracking = false;
    private boolean timedOut = false;
    private double stallX, stallY, stallHeading;

    private Pose2D pose;

    public gtp_helper(DcMotor leftDrive, DcMotor rightDrive, GoBildaPinpointDriver pinpoint) {
        this.leftDrive = leftDrive;
        this.rightDrive = rightDrive;
        this.pinpoint = pinpoint;
    }

    public static gtp_helper fromHardwareMap(HardwareMap hardwareMap,
                                             String leftName,
                                             String rightName,
                                             String pinpointName,
                                             double podOffsetX_mm,
                                             double podOffsetY_mm) {
        DcMotor left = hardwareMap.get(DcMotor.class, leftName);
        DcMotor right = hardwareMap.get(DcMotor.class, rightName);

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, pinpointName);
        pinpoint.setOffsets(podOffsetX_mm, podOffsetY_mm, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        return new gtp_helper(left, right, pinpoint);
    }

    public boolean update(double targetX_mm, double targetY_mm) {
        pinpoint.update();
        pose = pinpoint.getPosition();

        double currentX = pose.getX(DistanceUnit.MM);
        double currentY = pose.getY(DistanceUnit.MM);
        double currentHeading = pose.getHeading(AngleUnit.DEGREES);

        double dx = targetX_mm - currentX;
        double dy = targetY_mm - currentY;
        double distance = Math.hypot(dx, dy);

        if (distance <= DISTANCE_TOLERANCE_MM) {
            timedOut = false;
            stallTracking = false;
            stop();
            return true;
        }

        if (isStalled(currentX, currentY, currentHeading)) {
            timedOut = true;
            stallTracking = false;
            stop();
            return true;
        }

        double headingError = normalizeAngle(
                Math.toDegrees(Math.atan2(dy, dx)) - currentHeading);

        double driveSign = 1.0;
        if (headingError > 90.0) {
            headingError -= 180.0;
            driveSign = -1.0;
        } else if (headingError < -90.0) {
            headingError += 180.0;
            driveSign = -1.0;
        }

        double drivePower;
        double turnPower;

        if (Math.abs(headingError) > HEADING_LOCK_DEG) {
            drivePower = 0.0;
            turnPower = applyMinPower(KP_TURN * headingError, MIN_TURN_POWER);
        } else {
            drivePower = applyMinPower(driveSign * KP_DRIVE * distance, MIN_DRIVE_POWER);
            turnPower = Math.abs(headingError) < HEADING_DEADBAND_DEG
                    ? 0.0
                    : KP_TURN * headingError * TRANSLATE_TURN_SCALE;
        }

        setDriveTurn(drivePower, turnPower);
        return false;
    }

    public boolean runToPoint(LinearOpMode opMode, double targetX_mm, double targetY_mm) {
        while (opMode.opModeIsActive() && !update(targetX_mm, targetY_mm)) {
            opMode.idle();
        }
        stop();
        return !timedOut;
    }

    public boolean isTimedOut() {
        return timedOut;
    }

    public void stop() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void resetPose() {
        pinpoint.resetPosAndIMU();
        stallTracking = false;
        timedOut = false;
    }

    public Pose2D getPose() {
        return pose;
    }

    public double getX() {
        return pose == null ? 0 : pose.getX(DistanceUnit.MM);
    }

    public double getY() {
        return pose == null ? 0 : pose.getY(DistanceUnit.MM);
    }

    public double getHeading() {
        return pose == null ? 0 : pose.getHeading(AngleUnit.DEGREES);
    }

    private boolean isStalled(double x, double y, double heading) {
        if (!stallTracking) {
            stallX = x;
            stallY = y;
            stallHeading = heading;
            stallTimer.reset();
            stallTracking = true;
            return false;
        }

        boolean moved = Math.hypot(x - stallX, y - stallY) > STALL_DISTANCE_MM
                || Math.abs(normalizeAngle(heading - stallHeading)) > STALL_HEADING_DEG;

        if (moved) {
            stallX = x;
            stallY = y;
            stallHeading = heading;
            stallTimer.reset();
            return false;
        }

        return stallTimer.seconds() > STALL_TIMEOUT_S;
    }

    private void setDriveTurn(double drive, double turn) {
        drive = clamp(drive, -MAX_POWER, MAX_POWER);
        turn = clamp(turn, -MAX_POWER, MAX_POWER);

        double left = drive - turn;
        double right = drive + turn;

        double maxMag = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)));
        leftDrive.setPower(left / maxMag);
        rightDrive.setPower(right / maxMag);
    }

    private static double applyMinPower(double power, double minPower) {
        if (power == 0) return 0;
        return Math.abs(power) < minPower ? Math.copySign(minPower, power) : power;
    }

    private static double normalizeAngle(double angleDeg) {
        while (angleDeg > 180) angleDeg -= 360;
        while (angleDeg < -180) angleDeg += 360;
        return angleDeg;
    }

    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}