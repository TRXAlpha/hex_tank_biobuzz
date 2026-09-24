package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class headinghelper_centruderotatie {

    public static double HEADING_TOLERANCE_DEG = 0.4;
    public static double KP = 0.55;
    public static double TANH_SCALE = 2.0;
    public static double KS = 0.2;
    public static double MAX_POWER = 1.0;
    public static double NOMINAL_VOLTAGE = 12.5;
    public static double RIGHT_TRIM = 0.017;

    public static double COR_FORWARD_MM = -100;
    public static double COR_LEFT_MM = 0;
    public static double TRACK_WIDTH_MM = 350;
    public static double KP_DRIFT = 0.008;
    public static double MAX_DRIFT_POWER = 0.3;

    private final DcMotorEx leftDrive;
    private final DcMotorEx rightDrive;
    private final GoBildaPinpointDriver pinpoint;
    private final VoltageSensor voltageSensor;

    private double heading;
    private double error;
    private double turnPower;
    private double drivePower;

    private boolean anchorSet;
    private double anchorX;
    private double anchorY;
    private double forwardDrift;
    private double lateralDrift;

    public headinghelper_centruderotatie(HardwareMap hardwareMap,
                                         DcMotorEx leftDrive,
                                         DcMotorEx rightDrive,
                                         GoBildaPinpointDriver pinpoint) {
        this.leftDrive = leftDrive;
        this.rightDrive = rightDrive;
        this.pinpoint = pinpoint;
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public static headinghelper_centruderotatie fromHardwareMap(HardwareMap hardwareMap,
                                                                String leftName,
                                                                String rightName,
                                                                String pinpointName,
                                                                double podOffsetX_mm,
                                                                double podOffsetY_mm) {
        DcMotorEx left = hardwareMap.get(DcMotorEx.class, leftName);
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, rightName);

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

        return new headinghelper_centruderotatie(hardwareMap, left, right, pinpoint);
    }

    public void resetAnchor() {
        anchorSet = false;
    }

    public boolean update(double targetHeadingDeg) {
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        heading = pose.getHeading(AngleUnit.DEGREES);

        double h = Math.toRadians(heading);
        double cos = Math.cos(h);
        double sin = Math.sin(h);

        double corX = pose.getX(DistanceUnit.MM) + COR_FORWARD_MM * cos - COR_LEFT_MM * sin;
        double corY = pose.getY(DistanceUnit.MM) + COR_FORWARD_MM * sin + COR_LEFT_MM * cos;

        if (!anchorSet) {
            anchorX = corX;
            anchorY = corY;
            anchorSet = true;
        }

        double dx = anchorX - corX;
        double dy = anchorY - corY;
        forwardDrift = dx * cos + dy * sin;
        lateralDrift = -dx * sin + dy * cos;

        error = normalizeAngle(targetHeadingDeg - heading);

        if (Math.abs(error) <= HEADING_TOLERANCE_DEG) {
            turnPower = 0;
            drivePower = 0;
            stop();
            return true;
        }

        double voltageComp = NOMINAL_VOLTAGE / voltageSensor.getVoltage();

        double raw = Math.tanh(Math.toRadians(error) * TANH_SCALE) * KP;
        turnPower = (raw + Math.copySign(KS, raw)) * voltageComp;

        double feedforward = turnPower * 2.0 * COR_LEFT_MM / TRACK_WIDTH_MM;
        double correction = Range.clip(forwardDrift * KP_DRIFT, -MAX_DRIFT_POWER, MAX_DRIFT_POWER) * voltageComp;
        drivePower = feedforward + correction;

        double left = drivePower - turnPower;
        double right = drivePower + turnPower + RIGHT_TRIM;

        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > MAX_POWER) {
            left = left / max * MAX_POWER;
            right = right / max * MAX_POWER;
        }

        leftDrive.setPower(left);
        rightDrive.setPower(right);
        return false;
    }

    public void runToHeading(LinearOpMode opMode, double targetHeadingDeg) {
        resetAnchor();
        while (opMode.opModeIsActive() && !update(targetHeadingDeg)) {
            opMode.idle();
        }
        stop();
    }

    public void stop() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public double getHeading() {
        return heading;
    }

    public double getError() {
        return error;
    }

    public double getTurnPower() {
        return turnPower;
    }

    public double getDrivePower() {
        return drivePower;
    }

    public double getForwardDrift() {
        return forwardDrift;
    }

    public double getLateralDrift() {
        return lateralDrift;
    }

    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    private static double normalizeAngle(double angleDeg) {
        while (angleDeg > 180) angleDeg -= 360;
        while (angleDeg < -180) angleDeg += 360;
        return angleDeg;
    }
}