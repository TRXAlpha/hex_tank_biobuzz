package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Smooth Drive Tuner (Pinpoint)", group = "Tuning")
@Config
public class drivetanh extends OpMode {

    // Tunable gains (use FTC Dashboard)
    public static double kP = 0.25;
    public static double kD = 0.012;          // ← start here, raise if still overshoots
    public static double tanhScale = 0.15;
    public static double kS = 0.15;
    public static double maxPower = 0.85;

    public static double targetX = 100;       // cm
    public static int maxCycles = 3;
    public static double arrivalTolerance = 0.5; // cm

    DcMotor stanga, dreapta;
    GoBildaPinpointDriver pinpoint;

    private double currentTarget = 0;
    private boolean goingToTarget = true;
    private int completedCycles = 0;
    private boolean finished = false;

    @Override
    public void init() {
        stanga = hardwareMap.get(DcMotor.class, "stanga");
        dreapta = hardwareMap.get(DcMotor.class, "dreapta");

        stanga.setDirection(DcMotor.Direction.REVERSE);
        dreapta.setDirection(DcMotor.Direction.FORWARD);
        stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(0, 55, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.resetPosAndIMU();

        currentTarget = targetX;
        goingToTarget = true;
        completedCycles = 0;
        finished = false;
    }

    @Override
    public void loop() {
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        double currentX = pose.getX(DistanceUnit.CM);

        // Velocity in cm/s (Pinpoint reports mm/s internally)
        double velocityX = pinpoint.getVelX(DistanceUnit.CM);   // cm/s

        double error = currentTarget - currentX;
        double power = 0;

        if (!finished) {
            // P term with tanh soft saturation
            double pTerm = Math.tanh(error * tanhScale) * kP;

            // D term: oppose velocity (derivative of error = -velocity)
            double dTerm = -kD * velocityX;

            double raw = pTerm + dTerm;

            if (Math.abs(error) < arrivalTolerance) {
                power = 0;
                // Arrived → switch direction
                if (goingToTarget) {
                    currentTarget = 0;
                    goingToTarget = false;
                } else {
                    completedCycles++;
                    if (completedCycles >= maxCycles) {
                        finished = true;
                        currentTarget = 0;
                    } else {
                        currentTarget = targetX;
                        goingToTarget = true;
                    }
                }
            } else {
                // static friction compensation
                power = raw + Math.copySign(kS, raw);
            }

            power = Range.clip(power, -maxPower, maxPower);
        }

        stanga.setPower(power);
        dreapta.setPower(power + 0.017);   // small bias you already had

        // Telemetry
        telemetry.addData("Current Target", "%.0f cm", currentTarget);
        telemetry.addData("Current X", "%.1f cm", currentX);
        telemetry.addData("Error", "%.1f cm", error);
        telemetry.addData("Velocity X", "%.1f cm/s", velocityX);
        telemetry.addData("Power", "%.3f", power);
        telemetry.addData("Going to target?", goingToTarget);
        telemetry.addData("Cycles done", "%d / %d", completedCycles, maxCycles);
        telemetry.addData("Finished?", finished);
        telemetry.addData("--- Gains ---", "");
        telemetry.addData("kP", kP);
        telemetry.addData("kD", kD);
        telemetry.addData("tanhScale", tanhScale);
        telemetry.addData("kS", kS);
        telemetry.update();
    }
}