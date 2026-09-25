package org.firstinspires.ftc.teamcode.auto.helpere;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class AutoHelper {

    // ===================== COR =====================
    // Pozitia COR fata de punctul urmarit, in cadrul robotului
    public static double COR_FORWARD_MM = -100;
    public static double COR_LEFT_MM = 0;

    // ===================== GO-TO-POINT =====================
    public static double DISTANCE_TOLERANCE_MM = 10;
    public static double HEADING_LOCK_DEG = 10;
    public static double HEADING_DEADBAND_DEG = 1.5;
    public static double KP_DRIVE = 0.0035;
    public static double KP_TURN = 0.02;
    public static double MAX_POWER = 0.75;
    public static double MIN_DRIVE_POWER = 0.4;
    public static double MIN_TURN_POWER = 0.4;
    public static double TRANSLATE_TURN_SCALE = 0.5;

    // ===================== HEADING PE LOC =====================
    public static double HEADING_KP = 0.55;
    public static double HEADING_TANH_SCALE = 2.0;
    public static double HEADING_KS = 0.18;
    public static double HEADING_KD = 0.002;
    public static double HEADING_DEADBAND = 0.6;
    public static double HEADING_SETTLE_RATE = 15;
    public static double HEADING_MAX_POWER = 0.9;
    public static double HEADING_MIN_POWER = 0.4;
    public static double NOMINAL_VOLTAGE = 12.5;
    public static double KS_MIN_ERROR_DEG = 3.0;

    public static DcMotor stanga;
    public static DcMotor dreapta;
    private final GoBildaPinpointDriver pinpoint;
    private final VoltageSensor voltageSensor;
    private final ElapsedTime timer = new ElapsedTime();

    private Pose2D pose;

    private boolean doneXY = false;
    private boolean doneHeading = false;

    private double lastHeading;
    private double lastHeadingTime = -1;

    private double targetX = Double.NaN;
    private double targetY = Double.NaN;

    public AutoHelper(DcMotor stanga, DcMotor dreapta,
                      GoBildaPinpointDriver pinpoint, HardwareMap hardwareMap) {
        this.stanga = stanga;
        this.dreapta = dreapta;
        this.pinpoint = pinpoint;
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public static AutoHelper fromHardwareMap(HardwareMap hardwareMap,
                                             String leftName, String rightName, String pinpointName,
                                             double podOffsetX_mm, double podOffsetY_mm) {
        DcMotor stanga = hardwareMap.get(DcMotor.class, leftName);
        DcMotor dreapta = hardwareMap.get(DcMotor.class, rightName);

        stanga.setDirection(DcMotorSimple.Direction.REVERSE);
        dreapta.setDirection(DcMotorSimple.Direction.FORWARD);
        stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, pinpointName);
        pinpoint.setOffsets(podOffsetX_mm, podOffsetY_mm, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        return new AutoHelper(stanga, dreapta, pinpoint, hardwareMap);
    }

    // =========================================================
    //                    GO-TO-POINT (COR)
    // =========================================================

    public void updateXY(double targetX_mm, double targetY_mm,
                         double finalHeadingDeg, Telemetry t) {
        refreshPose();
        targetX = targetX_mm;
        targetY = targetY_mm;
        lastHeadingTime = -1;

        double heading = pose.getHeading(DEGREES);
        double[] offsetNow = rotate(COR_FORWARD_MM, COR_LEFT_MM, heading);
        double[] offsetFinal = rotate(COR_FORWARD_MM, COR_LEFT_MM, finalHeadingDeg);

        double corX = pose.getX(DistanceUnit.MM) + offsetNow[0];
        double corY = pose.getY(DistanceUnit.MM) + offsetNow[1];

        double dx = targetX_mm + offsetFinal[0] - corX;
        double dy = targetY_mm + offsetFinal[1] - corY;
        double distance = Math.hypot(dx, dy);

        if (distance <= DISTANCE_TOLERANCE_MM) {
            stop();
            doneXY = true;
            return;
        }

        double headingError = normalizeAngle(Math.toDegrees(Math.atan2(dy, dx)) - heading);

        double driveSign = 1.0;
        if (headingError > 90.0) {
            headingError -= 180.0;
            driveSign = -1.0;
        } else if (headingError < -90.0) {
            headingError += 180.0;
            driveSign = -1.0;
        }

        t.addData("cor dx", dx);
        t.addData("cor dy", dy);
        t.addData("heading error", headingError);

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
        doneXY = false;
    }

    // =========================================================
    //               HEADING DOAR PE LOC
    // =========================================================

    public void updateHeading(double targetHeadingDeg, Telemetry t) {
        refreshPose();

        double current = pose.getHeading(DEGREES);
        double error = normalizeAngle(targetHeadingDeg - current);

        double now = timer.seconds();
        double rate = 0;
        if (lastHeadingTime >= 0 && now > lastHeadingTime) {
            rate = normalizeAngle(current - lastHeading) / (now - lastHeadingTime);
        }
        lastHeading = current;
        lastHeadingTime = now;

        t.addData("target heading", targetHeadingDeg);
        t.addData("current heading", current);
        t.addData("heading error", error);
        t.addData("heading rate", rate);
        if (!Double.isNaN(targetX)) {
            t.addData("point error", Math.hypot(
                    targetX - pose.getX(DistanceUnit.MM),
                    targetY - pose.getY(DistanceUnit.MM)));
        }

        if (Math.abs(error) <= HEADING_DEADBAND) {
            stop();
            doneHeading = Math.abs(rate) <= HEADING_SETTLE_RATE;
            return;
        }

        double power = Math.tanh(Math.toRadians(error) * HEADING_TANH_SCALE) * HEADING_KP;

        if (Math.abs(error) > KS_MIN_ERROR_DEG) {
            power += Math.copySign(HEADING_KS, error);
        }

        power -= HEADING_KD * rate;
        power *= NOMINAL_VOLTAGE / voltageSensor.getVoltage();
        if (Math.abs(rate) < HEADING_SETTLE_RATE && Math.abs(power) < HEADING_MIN_POWER) {
            power = Math.copySign(HEADING_MIN_POWER, error);
        }
        power = Range.clip(power, -HEADING_MAX_POWER, HEADING_MAX_POWER);

        stanga.setPower(-power);
        dreapta.setPower(power);
        doneHeading = false;
    }

    public boolean lap(double targetHeadingDeg, double targetX_mm,
                       double targetY_mm, Telemetry t) {
        if (!doneXY) {
            updateXY(targetX_mm, targetY_mm, targetHeadingDeg, t);
        } else if (!doneHeading) {
            updateHeading(targetHeadingDeg, t);
        }
        return doneXY && doneHeading;
    }

    public void resetMove() {
        pinpoint.update();
        doneXY = false;
        doneHeading = false;
        lastHeadingTime = -1;
    }

    // =========================================================
    //                    UTILITARE
    // =========================================================

    public void refreshPose() {
        pinpoint.update();
        pose = pinpoint.getPosition();
    }

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
        return pose == null ? 0 : pose.getHeading(DEGREES);
    }

    public boolean getStatusXY() {
        return doneXY;
    }

    public boolean getStatusHeading() {
        return doneHeading;
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

    private static double[] rotate(double forward, double left, double headingDeg) {
        double h = Math.toRadians(headingDeg);
        double cos = Math.cos(h);
        double sin = Math.sin(h);
        return new double[]{
                forward * cos - left * sin,
                forward * sin + left * cos
        };
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