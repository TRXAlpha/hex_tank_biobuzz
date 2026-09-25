package org.firstinspires.ftc.teamcode.kebab;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class HeadingController {

    // ===================== PARAMETRI TUNABILI (Dashboard) =====================
    public static double kP = 0.55;
    public static double tanhScale = 2.0;
    public static double kS = 0.18;               // static friction
    public static double maxPower = 0.85;
    public static double deadbandDeg = 0.5;       // sub această eroare → 0
    public static double nominalVoltage = 12.5;
    // ==========================================================================

    private final DcMotorEx left;
    private final DcMotorEx right;
    private final GoBildaPinpointDriver pinpoint;
    private final HardwareMap hw;

    private double targetHeading = 0.0;

    public HeadingController(HardwareMap hardwareMap,
                             String leftName,
                             String rightName,
                             String pinpointName) {
        this.hw = hardwareMap;

        left = hardwareMap.get(DcMotorEx.class, leftName);
        right = hardwareMap.get(DcMotorEx.class, rightName);

        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE); // important! (sau invers, depinde de cablare)

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, pinpointName);
        pinpoint.setOffsets(0, 55, DistanceUnit.MM); // ← pune offset-urile tale reale
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.resetPosAndIMU();
    }

    public void setTarget(double degrees) {
        this.targetHeading = degrees;
    }

    public double getTarget() {
        return targetHeading;
    }

    public double getCurrentHeading() {
        pinpoint.update();
        return pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
    }

    /**
     * Apelează în loop().
     * Returnează puterea de rotație aplicată (pozitiv = sens antiorar de obicei).
     */
    public double update() {
        pinpoint.update();

        double battery = hw.voltageSensor.iterator().next().getVoltage();
        double current = pinpoint.getPosition().getHeading(AngleUnit.DEGREES);

        double error = AngleUnit.normalizeDegrees(targetHeading - current); // important!

        // Controller tanh + kS
        double raw = Math.tanh(Math.toRadians(error) * tanhScale) * kP;

        double power;
        if (Math.abs(error) < deadbandDeg) {
            power = 0;
        } else {
            power = raw + Math.copySign(kS, raw);
        }

        // Compensare tensiune
        double scale = nominalVoltage / battery;
        power *= scale;

        power = Range.clip(power, -maxPower, maxPower);

        // Puteri OPUSĂ pentru rotație pe loc
        left.setPower(power);
        right.setPower(-power);   // ← aici e diferența importantă

        return power;
    }

    public void stop() {
        left.setPower(0);
        right.setPower(0);
    }

    public void addTelemetry(Telemetry t) {
        double current = getCurrentHeading();
        double error = AngleUnit.normalizeDegrees(targetHeading - current);
        double battery = hw.voltageSensor.iterator().next().getVoltage();

        t.addData("Target", "%.1f°", targetHeading);
        t.addData("Current", "%.1f°", current);
        t.addData("Error", "%.2f°", error);
        t.addData("Battery", "%.2f V", battery);
        t.addData("kP / tanhScale / kS", "%.2f / %.1f / %.2f", kP, tanhScale, kS);
    }
}