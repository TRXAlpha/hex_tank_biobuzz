package org.firstinspires.ftc.teamcode.tuning;

import static java.lang.Math.atan;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@Config
@Autonomous(name = "Fluid Movement Controller", group = "Test")
public class pozitionare extends OpMode {

    // ========== heading pid ==========
    public static double kP = 0.55;
    public static double tanhScale = 2.0;
    public static double kS = 0.2;
    public static double maxPower = 1.0;

    // ========== drive pid ==========
    public static double kPdrive = 0.06;
    public static double kD = 0.0075;
    public static double maxDrive = 1.0;
    public static double posTolerance = 0.2;

    // ========== target ==========
    public static double targetX = 0;
    public static double targetY = 0;
    public static double targetHeading = 0;

    private DcMotorEx stanga, dreapta;
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        stanga  = hardwareMap.get(DcMotorEx.class, "stanga");
        dreapta = hardwareMap.get(DcMotorEx.class, "dreapta");

        stanga.setDirection(DcMotor.Direction.REVERSE);
        dreapta.setDirection(DcMotor.Direction.FORWARD);
        stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(0, 55, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();
    }

    private boolean driveActive = false;

    @Override
    public void loop() {
        pinpoint.update();



        driveActive = true;
        if (driveActive && drivePID()) driveActive = false;


        telemetry.addData("Target", "X=%.0f Y=%.0f H=%.0f", targetX, targetY, targetHeading);
        telemetry.addData("Pose",   "X=%.1f Y=%.1f H=%.1f", pinpoint.getPosX(DistanceUnit.CM), pinpoint.getPosY(DistanceUnit.CM), pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.update();

    }


    public boolean drivePID() {
        double x = pinpoint.getPosX(DistanceUnit.CM);
        double velocityX = pinpoint.getVelX(DistanceUnit.CM);

        double errorX = targetX - x; // signed, keep sign

        if (Math.abs(errorX) <= posTolerance) {
            stanga.setPower(0);
            dreapta.setPower(0);
            return true;
        }

        double raw = kPdrive * errorX - kD * velocityX; // signed throughout
        raw += Math.copySign(kS, raw);

        double drive = Range.clip(raw, -maxDrive, maxDrive);

        stanga.setPower(drive);
        dreapta.setPower(drive + 0.017);

        return false;
    }    public double headingPID(double targetHeadingRad) {
        pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);

        Pose2D pose = pinpoint.getPosition();
        double currentHeading = pose.getHeading(AngleUnit.RADIANS);

        double error = targetHeadingRad - currentHeading;

        // tanh controller
        double raw = Math.tanh(error * tanhScale) * kP;

        double power;
        if (Math.abs(error) < 0.4) power = 0;
            else power = raw + Math.copySign(kS, raw);

        power = Range.clip(power, -maxPower, maxPower);

        stanga.setPower(power);
        dreapta.setPower(power + 0.017);

        return error;
    }
      // helpers
    private double angleWrap(double a) {
        while (a >  180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }
}