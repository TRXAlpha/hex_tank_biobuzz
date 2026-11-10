package org.firstinspires.ftc.teamcode.auto.helpere;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@Config
public class AutoHelper {

    // ===================== COR =====================
    // Pozitia COR fata de punctul urmarit, in cadrul robotului
    public static double COR_FORWARD_MM = 0;
    public static double COR_LEFT_MM = 0;

    // ===================== GO-TO-POINT =====================
    // Valori din fataspatetuning, trecute din cm in mm
    public static double DISTANCE_TOLERANCE_MM = 15;
    public static double MIN_MOVE_MM = 80;
    public static double HEADING_LOCK_DEG = 3;
    public static double KP_DRIVE = 0.0022;
    public static double MAX_POWER = 0.5;
    public static double MIN_DRIVE_POWER = 0.15;
    public static double KP_TURN = 0.02;
    public static double MAX_TURN = 0.3;
    public static double LOOKAHEAD_MM = 200;
    public static double MAX_CORRECTION_DEG = 20;
    public static double FADE_DISTANCE_MM = 150;

    // ===================== HEADING PE LOC =====================
    // Valori din headingPID360
    public static double HEADING_KP = 0.55;
    public static double HEADING_TANH_SCALE = 2.0;
    public static double HEADING_KS = 0.2;
    public static double HEADING_KD = 0.002;
    public static double HEADING_DEADBAND = 0.4;
    public static double HEADING_SETTLE_RATE = 15;
    public static double HEADING_MAX_POWER = 1.0;
    public static double NOMINAL_VOLTAGE = 12.5;

    public static DcMotor stanga;
    public static DcMotor dreapta;
    private final GoBildaPinpointDriver pinpoint;
    private final VoltageSensor voltageSensor;

    private Pose2D pose;

    private boolean doneXY = false;
    private boolean doneHeading = false;

    private double targetX = Double.NaN;
    private double targetY = Double.NaN;

    // Linia pe care o urmeaza miscarea curenta
    private boolean lineReady = false;
    private boolean facing = false;
    private double lineStartX;
    private double lineStartY;
    private double lineHeading;

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
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
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
        if (targetX_mm != targetX || targetY_mm != targetY) {
            lineReady = false;
        }
        targetX = targetX_mm;
        targetY = targetY_mm;

        double heading = pose.getHeading(DEGREES);
        double[] offsetNow = rotate(COR_FORWARD_MM, COR_LEFT_MM, heading);
        double[] offsetFinal = rotate(COR_FORWARD_MM, COR_LEFT_MM, finalHeadingDeg);

        double corX = pose.getX(DistanceUnit.MM) + offsetNow[0];
        double corY = pose.getY(DistanceUnit.MM) + offsetNow[1];

        double goalX = targetX_mm + offsetFinal[0];
        double goalY = targetY_mm + offsetFinal[1];

        if (!lineReady) {
            startLine(corX, corY, goalX, goalY, heading, finalHeadingDeg);
        }

        double cos = Math.cos(Math.toRadians(lineHeading));
        double sin = Math.sin(Math.toRadians(lineHeading));
        double along = (goalX - corX) * cos + (goalY - corY) * sin;
        double cross = (corX - lineStartX) * -sin + (corY - lineStartY) * cos;

        t.addData("along", along);
        t.addData("cross", cross);

        if (Math.abs(along) <= DISTANCE_TOLERANCE_MM) {
            stop();
            doneXY = true;
            return;
        }

        doneXY = false;

        if (facing) {
            double faceError = normalizeAngle(lineHeading - heading);
            t.addData("heading error", faceError);
            if (Math.abs(faceError) > HEADING_LOCK_DEG) {
                setTurn(headingPower(faceError));
                return;
            }
            facing = false;
        }

        double direction = Math.signum(along);

        // Tinteste un punct de pe linie in fata; slabeste aproape de tinta, oprit nu mai poate corecta
        double correction = Math.toDegrees(Math.atan2(-cross, LOOKAHEAD_MM));
        double fade = Math.min(1.0, Math.abs(along) / FADE_DISTANCE_MM);
        double desiredHeading = lineHeading
                + Range.clip(correction, -MAX_CORRECTION_DEG, MAX_CORRECTION_DEG) * direction * fade;
        double headingError = normalizeAngle(desiredHeading - heading);

        t.addData("heading error", headingError);

        double turn = Range.clip(KP_TURN * headingError, -MAX_TURN, MAX_TURN);
        double drive = direction * Range.clip(Math.abs(KP_DRIVE * along), MIN_DRIVE_POWER, MAX_POWER);

        setDriveTurn(drive, turn);
    }

    private void startLine(double corX, double corY, double goalX, double goalY,
                           double heading, double finalHeading) {
        lineStartX = corX;
        lineStartY = corY;
        lineReady = true;

        // La o miscare atat de mica doar s-ar invarti, asa ca merge fata-spate pe directia curenta
        if (Math.hypot(goalX - corX, goalY - corY) <= MIN_MOVE_MM) {
            lineHeading = heading;
            facing = false;
            return;
        }

        double bearing = Math.toDegrees(Math.atan2(goalY - corY, goalX - corX));
        double reverse = normalizeAngle(bearing + 180.0);

        // Alege sensul cu cea mai putina rotire in total; rotirea de la final nu mai e corectata
        lineHeading = totalTurn(heading, reverse, finalHeading) < totalTurn(heading, bearing, finalHeading)
                ? reverse
                : bearing;

        facing = true;
    }

    // =========================================================
    //               HEADING DOAR PE LOC
    // =========================================================

    public void updateHeading(double targetHeadingDeg, Telemetry t) {
        refreshPose();

        double current = pose.getHeading(DEGREES);
        double error = normalizeAngle(targetHeadingDeg - current);
        double rate = headingRate();

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

        setTurn(headingPower(error));
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
        lineReady = false;
        facing = false;
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

    private double headingRate() {
        return pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
    }

    private double headingPower(double errorDeg) {
        double raw = Math.tanh(Math.toRadians(errorDeg) * HEADING_TANH_SCALE) * HEADING_KP;
        double power = (raw + Math.copySign(HEADING_KS, raw) - HEADING_KD * headingRate())
                * NOMINAL_VOLTAGE / voltageSensor.getVoltage();
        return Range.clip(power, -HEADING_MAX_POWER, HEADING_MAX_POWER);
    }

    private void setTurn(double power) {
        stanga.setPower(-power);
        dreapta.setPower(power);
    }

    private void setDriveTurn(double drive, double turn) {
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

    private static double totalTurn(double heading, double lineHeading, double finalHeading) {
        return Math.abs(normalizeAngle(lineHeading - heading))
                + Math.abs(normalizeAngle(finalHeading - lineHeading));
    }

    private static double normalizeAngle(double angleDeg) {
        while (angleDeg > 180) angleDeg -= 360;
        while (angleDeg < -180) angleDeg += 360;
        return angleDeg;
    }
}
