package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
@TeleOp(name = "miscare", group = "Test")
public class pozitionare2 extends OpMode {

    // ========== Tunable target (geometric center) ==========
    public static double targetX       = 100.0;   // cm
    public static double targetY       = -40.0;  // cm
    public static double targetHeading = 0.0;    // degrees
    // ========== Heading gains ==========
    public static double kPturn    = 0.55;
    public static double tanhScale = 2.0;
    public static double kSturn    = 0.20;
    public static double maxTurn   = 0.50;
    // Only correct the forward component (most common with rear/front grip difference)
    // and keep it tiny
    public static double kPhold   = 0.04;   // ← tune very low
    public static double maxHold  = 0.18;    // hard limit – never more than this
    public static double posTol     = 1.0;
    public static double approachDist = 1.0;

    public static boolean reached = false;
    private DcMotor stanga, dreapta;
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        stanga  = hardwareMap.get(DcMotor.class, "stanga");
        dreapta = hardwareMap.get(DcMotor.class, "dreapta");

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

        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void loop() {
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();

        double x       = pose.getX(DistanceUnit.CM);
        double y       = pose.getY(DistanceUnit.CM);
        double heading = pose.getHeading(AngleUnit.DEGREES);

        // ============================================================
        // Desired heading (always the final target for now)
        // ============================================================
        double desiredHeading;

        // ============================================================
        // Gentle position hold (fights COR drift while turning)
        // Very low gains + hard clamp so it can never run across the field
        // ============================================================
        double dx = targetX - x;
        double dy = targetY - y;
        double distance = Math.hypot(dx, dy);

        if (distance > approachDist) {
            // Far → point toward the target point
            desiredHeading = Math.toDegrees(Math.atan2(dy, dx));
        } else {
            // Close → go to the final requested heading
            reached=true;
            desiredHeading = targetHeading;
        }

        double headingError   = angleWrap(desiredHeading - heading);
        double headingErrorRad = Math.toRadians(headingError);

        // Convert field error → robot frame
        double headingRad = Math.toRadians(heading);
        double errorForward = dx * Math.cos(headingRad) + dy * Math.sin(headingRad);
        double errorStrafe  = -dx * Math.sin(headingRad) + dy * Math.cos(headingRad);

        double drive = 0;
        if (distance > posTol && !reached) {
            double cosTerm = Math.max(0.0, Math.cos(headingErrorRad));
            double raw = kPhold * errorForward * cosTerm;

            drive = Range.clip(raw, -maxHold, maxHold);
        }

        // Optional: also fight a little lateral drift (usually smaller)
        double strafe = Range.clip(0.008 * errorStrafe, -0.10, 0.10);
        // (you can ignore strafe for now on a differential drive)

        // ============================================================
        // Heading controller (unchanged)
        // ============================================================
        double rawTurn = Math.tanh(headingErrorRad * tanhScale) * kPturn;
        double turn = 0;
        if (Math.abs(headingError) > 0.7) {
            turn = rawTurn + Math.copySign(kSturn, rawTurn) + strafe;
        }
        turn = Range.clip(turn, -maxTurn, maxTurn);


        // Mix
        double left  = drive - turn;
        double right = drive + turn;

        double max = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)));
        setPower(left / max, right / max);

        // Telemetry
        telemetry.addData("Target", "X=%.0f Y=%.0f H=%.0f", targetX, targetY, targetHeading);
        telemetry.addData("Pose",   "X=%.1f Y=%.1f H=%.1f", x, y, heading);
        telemetry.addData("Error",  "fwd=%.1f  head=%.1f", errorForward, headingError);
        telemetry.addData("Drive / Turn", "%.2f / %.2f", drive, turn);
        telemetry.addData("loopTime",pinpoint.getLoopTime());
        telemetry.addData("freq",pinpoint.getFrequency());
        telemetry.addData("deviceID",pinpoint.getDeviceID());
        telemetry.update();
    }

    private void setPower(double left, double right) {
        stanga.setPower(left);
        dreapta.setPower(right + 0.017);
    }

    private double angleWrap(double a) {
        while (a >  180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }
}