package org.firstinspires.ftc.teamcode.kebab;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


/// cod de pe github
/// cu valorile lui kebab
/// defapt pare ca merge si codul de dinainte, dar am aflat asta dupa ce am copiat codul
/// so folosim asta acum
/// plus reset ca sa poata fi folosit de mai multe ori in acelasi cod
/// pare ca merge destul de bine
/// la valori sunt schimbate DISTANCE_TOLERANCE_MM (10->3) si HEADING_DEADBAND (0.6->3)
/// D pe toate locurile unde era deja P
@Config
public class gtpHelperKebabCuResetCuD {

    // ===================== GO-TO-POINT =====================

    public static double DISTANCE_TOLERANCE_MM = 3;
    public static double HEADING_LOCK_DEG = 10;
    public static double HEADING_DEADBAND_DEG = 1.5;

    public static double KP_DRIVE = 0.0035;
    public static double KD_DRIVE = 0.0003;

    public static double KP_TURN = 0.02;
    public static double KD_TURN = 0.0001;

    public static double MAX_POWER = 0.75;
    public static double MIN_DRIVE_POWER = 0.25;
    public static double MIN_TURN_POWER = 0.25;
    public static double TRANSLATE_TURN_SCALE = 0.5;

    // ===================== HEADING PE LOC =====================

    public static double HEADING_KP = 0.55;
    public static double HEADING_KD = 0.001;
    public static double HEADING_TANH_SCALE = 2.0;
    public static double HEADING_KS = 0.18;
    public static double HEADING_DEADBAND = 3;
    public static double HEADING_MAX_POWER = 0.7;
    public static double NOMINAL_VOLTAGE = 12.5;

    // Prag pentru aplicarea kS
    public static double KS_MIN_ERROR_DEG = 3.0;

    // =======================================================

    private final DcMotor stanga;
    private final DcMotor dreapta;
    private final GoBildaPinpointDriver pinpoint;
    private final HardwareMap hardwareMap;

    private Pose2D pose;

    private boolean doneXY = false;
    private boolean doneHeading = false;
    private double lastDistanceError = 0;
    private double lastHeadingErrorXY = 0;
    private double lastHeadingError = 0;
    private long lastTimeNs = 0;

    public gtpHelperKebabCuResetCuD(
            DcMotor stanga,
            DcMotor dreapta,
            GoBildaPinpointDriver pinpoint,
            HardwareMap hardwareMap) {

        this.stanga = stanga;
        this.dreapta = dreapta;
        this.pinpoint = pinpoint;
        this.hardwareMap = hardwareMap;
    }

    public static gtpHelperKebabCuResetCuD fromHardwareMap(
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

        return new gtpHelperKebabCuResetCuD(stanga, dreapta, pinpoint, hardwareMap);
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
            lastDistanceError = 0;
            lastHeadingErrorXY = 0;
            return;
        }

        double headingError = normalizeAngle(
                Math.toDegrees(Math.atan2(dy, dx))
                        - pose.getHeading(AngleUnit.DEGREES));

        // --- calcul dt (secunde) ---
        long now = System.nanoTime();
        double dt = (lastTimeNs == 0) ? 0.02 : (now - lastTimeNs) / 1e9;
        lastTimeNs = now;
        dt = Math.max(dt, 0.005); // protectie impotriva dt prea mic

        // --- derivata ---
        double dDistance = (distance - lastDistanceError) / dt;
        double dHeading  = (headingError - lastHeadingErrorXY) / dt;

        lastDistanceError = distance;
        lastHeadingErrorXY = headingError;

        t.addData("x error", dx);
        t.addData("y error", dy);
        t.addData("heading error", headingError);
        t.addData("dDistance", dDistance);
        t.addData("dHeading", dHeading);

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

            // P + D pe turn
            double turnRaw = KP_TURN * headingError + KD_TURN * dHeading;
            turnPower = applyMinPower(turnRaw, MIN_TURN_POWER);

        } else {

            // P + D pe drive
            double driveRaw = driveSign * (KP_DRIVE * distance + KD_DRIVE * dDistance);
            drivePower = applyMinPower(driveRaw, MIN_DRIVE_POWER);

            // P + D pe turn (scalat)
            if (Math.abs(headingError) < HEADING_DEADBAND_DEG) {
                turnPower = 0.0;
            } else {
                turnPower = (KP_TURN * headingError + KD_TURN * dHeading) * TRANSLATE_TURN_SCALE;
            }
        }

        setDriveTurn(drivePower, turnPower);

        doneXY = false;
    }

    // =========================================================
    //               HEADING DOAR PE LOC
    // =========================================================

    public void updateHeading(
            double targetHeadingDeg,
            Telemetry t) {

        pinpoint.update();
        pose = pinpoint.getPosition();

        double current = pose.getHeading(AngleUnit.DEGREES);

        // Cea mai scurtă diferență unghiulară
        double error = normalizeAngle(targetHeadingDeg - current);

        t.addData("target heading", targetHeadingDeg);
        t.addData("current heading", current);
        t.addData("heading error", error);

        // ========== FIX IMPORTANT ==========
        // Folosim error-ul normalizat, NU diferența brută
        if (Math.abs(error) <= HEADING_DEADBAND) {
            stop();
            doneHeading = true;
            lastHeadingError = 0;
            return;
        }
        // ===================================

        // --- calcul dt ---
        long now = System.nanoTime();
        double dt = (lastTimeNs == 0) ? 0.02 : (now - lastTimeNs) / 1e9;
        lastTimeNs = now;
        dt = Math.max(dt, 0.005);

        double dError = (error - lastHeadingError) / dt;
        lastHeadingError = error;

        t.addData("dHeading", dError);

        // Controller tanh (P) + D
        double rawP = Math.tanh(Math.toRadians(error) * HEADING_TANH_SCALE) * HEADING_KP;
        double rawD = HEADING_KD * dError;

        double power = rawP + rawD;

        // kS doar la erori mai mari
        if (Math.abs(error) > KS_MIN_ERROR_DEG) {
            power = power + Math.copySign(HEADING_KS, power);
        }

        // Compensare tensiune
        double battery = hardwareMap.voltageSensor
                .iterator()
                .next()
                .getVoltage();

        double scale = NOMINAL_VOLTAGE / battery;
        power *= scale;

        power = Range.clip(power, -HEADING_MAX_POWER, HEADING_MAX_POWER);

        // Puteri opuse = rotație pe loc
        stanga.setPower(-power);
        dreapta.setPower(power);

        doneHeading = false;
    }

    // =========================================================
    //                    UTILITARE
    // =========================================================

    public void reset() {
        doneXY = false;
        doneHeading = false;

        // reset și stările de derivată ca să nu aibă spike la următorul apel
        lastDistanceError = 0;
        lastHeadingErrorXY = 0;
        lastHeadingError = 0;
        lastTimeNs = 0;
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

        while (angleDeg > 180)
            angleDeg -= 360;

        while (angleDeg < -180)
            angleDeg += 360;

        return angleDeg;
    }

    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}