package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class kebabAutoHelper {

    // ===================== GO-TO-POINT =====================

    public static double DISTANCE_TOLERANCE_MM = 15;
    public static double HEADING_LOCK_DEG = 12;
    public static double HEADING_DEADBAND_DEG = 2;
    public static double KP_DRIVE = 0.0030;
    public static double KP_TURN = 0.018;
    public static double MAX_POWER = 0.70;
    public static double MIN_DRIVE_POWER = 0.28;
    public static double MIN_TURN_POWER = 0.28;
    public static double TRANSLATE_TURN_SCALE = 0.45;

    // ===================== HEADING + HOLD XY =====================

    public static double HEADING_KP = 0.38;
    public static double HEADING_TANH_SCALE = 1.8;
    public static double HEADING_KS = 0.12;
    public static double HEADING_DEADBAND = 1.2;
    public static double HEADING_MAX_POWER = 0.55;
    public static double NOMINAL_VOLTAGE = 12.5;

    // Prag pentru aplicarea kS
    public static double KS_MIN_ERROR_DEG = 4.0;

    // Corecție de poziție în timpul rotației
    public static double HOLD_XY_KP = 0.0038;
    public static double HOLD_XY_MAX_POWER = 0.32;
    public static double HOLD_XY_TOLERANCE_MM = 12;

    // =======================================================

    private final DcMotor stanga;
    private final DcMotor dreapta;
    private final GoBildaPinpointDriver pinpoint;
    private final HardwareMap hardwareMap;

    private Pose2D pose;

    private boolean doneXY = false;
    private boolean doneHeading = false;

    public kebabAutoHelper(
            DcMotor stanga,
            DcMotor dreapta,
            GoBildaPinpointDriver pinpoint,
            HardwareMap hardwareMap) {

        this.stanga = stanga;
        this.dreapta = dreapta;
        this.pinpoint = pinpoint;
        this.hardwareMap = hardwareMap;
    }

    public static kebabAutoHelper fromHardwareMap(
            HardwareMap hardwareMap,
            String leftName,
            String rightName,
            String pinpointName,
            double podOffsetX_mm,
            double podOffsetY_mm) {

        DcMotor stanga = hardwareMap.get(DcMotor.class, leftName);
        DcMotor dreapta = hardwareMap.get(DcMotor.class, rightName);

        stanga.setDirection(DcMotorSimple.Direction.REVERSE);
        dreapta.setDirection(DcMotorSimple.Direction.FORWARD);

        stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        GoBildaPinpointDriver pinpoint =
                hardwareMap.get(GoBildaPinpointDriver.class, pinpointName);

        pinpoint.setOffsets(podOffsetX_mm, podOffsetY_mm, DistanceUnit.MM);

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();

        return new kebabAutoHelper(stanga, dreapta, pinpoint, hardwareMap);
    }

    // =========================================================
    //                    GO-TO-POINT
    // =========================================================

    public void updateXY(
            double targetX_mm,
            double targetY_mm,
            Telemetry t) {

        pinpoint.update();
        pose = pinpoint.getPosition();

        double dx = targetX_mm - pose.getX(DistanceUnit.MM);
        double dy = targetY_mm - pose.getY(DistanceUnit.MM);

        double distance = Math.hypot(dx, dy);

        if (distance <= DISTANCE_TOLERANCE_MM) {
            stop();
            doneXY = true;
            return;
        }

        double headingError = normalizeAngle(
                Math.toDegrees(Math.atan2(dy, dx))
                        - pose.getHeading(AngleUnit.DEGREES));

        t.addData("x error", dx);
        t.addData("y error", dy);
        t.addData("heading error", headingError);

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
            drivePower = applyMinPower(
                    driveSign * KP_DRIVE * distance,
                    MIN_DRIVE_POWER);

            turnPower = Math.abs(headingError) < HEADING_DEADBAND_DEG
                    ? 0.0
                    : KP_TURN * headingError * TRANSLATE_TURN_SCALE;
        }

        setDriveTurn(drivePower, turnPower);
        doneXY = false;
    }

    // =========================================================
    //               HEADING + HOLD XY
    // =========================================================

    public void updateHeading(
            double targetHeadingDeg,
            double targetX_mm,
            double targetY_mm,
            Telemetry t) {

        pinpoint.update();
        pose = pinpoint.getPosition();

        double currentHeading = pose.getHeading(AngleUnit.DEGREES);
        double errorHeading = normalizeAngle(targetHeadingDeg - currentHeading);

        // === Corecție XY (hold position) ===
        double dx = targetX_mm - pose.getX(DistanceUnit.MM);
        double dy = targetY_mm - pose.getY(DistanceUnit.MM);
        double distance = Math.hypot(dx, dy);

        double drivePower = 0.0;

        if (distance > HOLD_XY_TOLERANCE_MM) {
            double absAngleToTarget = Math.toDegrees(Math.atan2(dy, dx));
            double relativeAngle = normalizeAngle(absAngleToTarget - currentHeading);

            // Proiecție pe axa robotului (forward / back)
            drivePower = HOLD_XY_KP * distance * Math.cos(Math.toRadians(relativeAngle));
            drivePower = Range.clip(drivePower, -HOLD_XY_MAX_POWER, HOLD_XY_MAX_POWER);
        }

        t.addData("target heading", targetHeadingDeg);
        t.addData("current heading", currentHeading);
        t.addData("heading error", errorHeading);
        t.addData("hold XY distance", distance);
        t.addData("hold drivePower", drivePower);

        // Deadband heading
        if (Math.abs(errorHeading) <= HEADING_DEADBAND) {
            if (distance <= HOLD_XY_TOLERANCE_MM) {
                stop();
                doneHeading = true;
                return;
            }
            // Doar corecție de poziție, fără turn
            setDriveTurn(drivePower, 0);
            doneHeading = false;
            return;
        }

        // Controller tanh + kS
        double raw = Math.tanh(Math.toRadians(errorHeading) * HEADING_TANH_SCALE) * HEADING_KP;

        double turnPower;
        if (Math.abs(errorHeading) > KS_MIN_ERROR_DEG) {
            turnPower = raw + Math.copySign(HEADING_KS, raw);
        } else {
            turnPower = raw;
        }

        // Compensare tensiune
        double battery = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double scale = NOMINAL_VOLTAGE / battery;
        turnPower *= scale;

        turnPower = Range.clip(turnPower, -HEADING_MAX_POWER, HEADING_MAX_POWER);

        // Combinăm drive (hold XY) + turn
        setDriveTurn(drivePower, turnPower);

        doneHeading = false;
    }

    // =========================================================
    //                       LAP
    // =========================================================

    public boolean lap(double targetHeadingDeg,
                       double targetX_mm,
                       double targetY_mm,
                       Telemetry t) {

        boolean doneXY = this.getStatusXY();
        boolean doneHeading = this.getStatusHeading();
        boolean done = doneXY && doneHeading;

        if (!doneXY) {
            this.updateXY(targetX_mm, targetY_mm, t);
        }
        if (!doneHeading && doneXY) {
            this.updateHeading(targetHeadingDeg, targetX_mm, targetY_mm, t);
        }
        return done;
    }

    // =========================================================
    //                    UTILITARE
    // =========================================================

    public void stop() {
        stanga.setPower(0);
        dreapta.setPower(0);
    }

    public void resetPose() {
        pinpoint.resetPosAndIMU();
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

    private void setDriveTurn(double drive, double turn) {
        drive = clamp(drive, -MAX_POWER, MAX_POWER);
        turn = clamp(turn, -MAX_POWER, MAX_POWER);

        double left = drive - turn;
        double right = drive + turn;

        double maxMag = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)));

        stanga.setPower(left / maxMag);
        dreapta.setPower(right / maxMag);
    }

    public boolean getStatusXY() {
        return doneXY;
    }

    public boolean getStatusHeading() {
        return doneHeading;
    }

    private static double applyMinPower(double power, double minPower) {
        if (power == 0) return 0;
        return Math.abs(power) < minPower
                ? Math.copySign(minPower, power)
                : power;
    }

    // =========================================================
    //               NORMALIZARE UNGHI
    // =========================================================

    private static double normalizeAngle(double angleDeg) {
        while (angleDeg > 180) angleDeg -= 360;
        while (angleDeg < -180) angleDeg += 360;
        return angleDeg;
    }

    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}