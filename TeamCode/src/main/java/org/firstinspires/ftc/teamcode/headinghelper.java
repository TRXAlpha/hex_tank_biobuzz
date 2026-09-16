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

@Config
public class headinghelper {

    public static double HEADING_TOLERANCE_DEG = 0.4;
    public static double KP = 0.55;
    public static double TANH_SCALE = 2.0;
    public static double KS = 0.2;
    public static double MAX_POWER = 1.0;
    public static double NOMINAL_VOLTAGE = 12.5;
    public static double RIGHT_TRIM = 0.017;

    private final DcMotorEx leftDrive;
    private final DcMotorEx rightDrive;
    private final GoBildaPinpointDriver pinpoint;
    private final VoltageSensor voltageSensor;

    private double heading;
    private double error;
    private double turnPower;

    public headinghelper(HardwareMap hardwareMap,
                         DcMotorEx leftDrive,
                         DcMotorEx rightDrive,
                         GoBildaPinpointDriver pinpoint) {
        this.leftDrive = leftDrive;
        this.rightDrive = rightDrive;
        this.pinpoint = pinpoint;
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public static headinghelper fromHardwareMap(HardwareMap hardwareMap,
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

        return new headinghelper(hardwareMap, left, right, pinpoint);
    }

    public boolean update(double targetHeadingDeg) {
        pinpoint.update();
        heading = pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
        error = normalizeAngle(targetHeadingDeg - heading);

        if (Math.abs(error) <= HEADING_TOLERANCE_DEG) {
            turnPower = 0;
            stop();
            return true;
        }

        double raw = Math.tanh(Math.toRadians(error) * TANH_SCALE) * KP;
        turnPower = (raw + Math.copySign(KS, raw)) * (NOMINAL_VOLTAGE / voltageSensor.getVoltage());

        double left = Range.clip(-turnPower, -MAX_POWER, MAX_POWER);
        double right = Range.clip(turnPower + RIGHT_TRIM, -MAX_POWER, MAX_POWER);

        leftDrive.setPower(left);
        rightDrive.setPower(right);
        return false;
    }

    public void runToHeading(LinearOpMode opMode, double targetHeadingDeg) {
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

    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    private static double normalizeAngle(double angleDeg) {
        while (angleDeg > 180) angleDeg -= 360;
        while (angleDeg < -180) angleDeg += 360;
        return angleDeg;
    }
}