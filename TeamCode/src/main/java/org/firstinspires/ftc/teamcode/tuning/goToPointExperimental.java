package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

// Driverul oficial goBILDA pentru Pinpoint. Trebuie adaugat separat in proiect:
// https://github.com/goBILDA-Official/FtcRobotController-Pinpoint
// (fisierul GoBildaPinpointDriver.java se pune in acelasi pachet teamcode / drivers)

/**
 * Deplaseaza robotul (tank drive, 2 motoare: stanga/dreapta) din (0,0) in (600,600) mm,
 * folosind pozitia raportata de Pinpoint Odometry Computer.
 *
 * IMPORTANT - lucruri pe care TREBUIE sa le verifici/ajustezi pe robotul tau real:
 *  1. Numele din hardwareMap ("motor1", "motor2", "pinpoint") trebuie sa corespunda
 *     exact cu numele configurate in Driver Station (Configure Robot).
 *  2. Sensul motoarelor (FORWARD/REVERSE) - daca robotul se roteste invers decat ar trebui,
 *     inverseaza directia unuia dintre motoare.
 *  3. Offset-urile pod-urilor de odometrie (podOffsetX_mm, podOffsetY_mm) - acestea NU sunt
 *     pozitia fizica a placutei Pinpoint, ci distanta de la centrul de rotatie al robotului
 *     pana la fiecare pod (X = pod-ul care masoara inainte/inapoi, Y = pod-ul care masoara
 *     stanga/dreapta). Valorile de mai jos sunt DOAR exemplu - masoara-le pe robotul tau.
 *  4. Sensul encoderelor de pe pod-uri (EncoderDirection) - daca X sau Y creste in sens
 *     opus fata de ce ar trebui, inverseaza.
 *  5. Coeficientii KP_TURN / KP_DRIVE si limitele de putere - se regleaza empiric.
 *
 * Conventie de coordonate/unghi folosita (conventia standard Pinpoint):
 *   - X pozitiv = inainte, Y pozitiv = stanga robotului, la pozitia de start.
 *   - Heading 0 = robotul e orientat pe directia +X initiala, unghi pozitiv = sens trigonometric (CCW).
 */
@Config
@Autonomous(name = "GoToPoint cu target heading", group = "Auto")
public class goToPointExperimental extends OpMode {

    // ----- Hardware -----
    DcMotor leftDrive, rightDrive;
    private GoBildaPinpointDriver pinpoint;
    public static double DISTANCE_TOLERANCE_MM = 10;
    public static double HEADING_LOCK_DEG = 10;
    public static double KP_DRIVE = 0.0035;
    public static double KP_TURN = 0.02;
    public static double MAX_POWER = 0.75;
    public static double MIN_DRIVE_POWER = 0.25;
    public static double MIN_TURN_POWER = 0.25;
    public static double FINAL_MIN_TURN_POWER = 0.08;
    public static double TARGET_X_MM = 1000;
    public static double TARGET_Y_MM = 1000;
    public static double HEADING_TOLERANCE_DEG = 5;
    public static double TARGET_HEADING_DEG = 0;
    // Vector from the Pinpoint-tracked point to the real center of rotation,
    // expressed in robot coordinates: +forward and +left. Initial estimates
    // below are derived from the latest simulated final-turn arc.
    public static double COR_FORWARD_MM = -100;
    public static double COR_LEFT_MM = 5;
    public static boolean terminat = false;
    boolean positionReached = false;
    @Override
    public void init() {

        // ---- Initializare motoare ----
        leftDrive  = hardwareMap.get(DcMotor.class, "stanga"); // Control Hub, portul 1
        rightDrive = hardwareMap.get(DcMotor.class, "dreapta"); // Control Hub, portul 2

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        double podOffsetX_mm = 50;
        double podOffsetY_mm = 0;

        pinpoint.setOffsets(podOffsetX_mm, podOffsetY_mm, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();
    }
    @Override
    public void start(){
        terminat = false;
        positionReached = false;
    }


    @Override
    public void loop(){
        if(!terminat) goToPoint(TARGET_X_MM, TARGET_Y_MM, TARGET_HEADING_DEG);

        if(terminat) {
            stopDrive();
            telemetry.addData("terminat", terminat);
        }
        telemetry.addData("X (mm)", "%.1f", pinpoint.getPosX(DistanceUnit.MM));
        telemetry.addData("Y (mm)", "%.1f", pinpoint.getPosY(DistanceUnit.MM));
        telemetry.addData("Heading (deg)", "%.1f", pinpoint.getHeading(AngleUnit.DEGREES));

        telemetry.update();
    }

    void goToPoint(double targetX, double targetY, double targetHeadingDeg) {
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();

        double currentX = pose.getX(DistanceUnit.MM);
        double currentY = pose.getY(DistanceUnit.MM);
        double currentHeadingDeg = pose.getHeading(AngleUnit.DEGREES);

        double dx = targetX - currentX;
        double dy = targetY - currentY;
        double distanceToTarget = Math.hypot(dx, dy);

        // If the tracked point is offset from the physical COR, it moves on an
        // arc during the final rotation. Aim Phase 1 at the point from which
        // rotating from the current heading to the final heading will place
        // the tracked point on the requested target.
        double[] corOffsetNow = fieldOffsetAtHeading(currentHeadingDeg);
        double[] corOffsetAtFinal = fieldOffsetAtHeading(targetHeadingDeg);
        double approachTargetX = targetX
                + corOffsetAtFinal[0] - corOffsetNow[0];
        double approachTargetY = targetY
                + corOffsetAtFinal[1] - corOffsetNow[1];
        double approachDx = approachTargetX - currentX;
        double approachDy = approachTargetY - currentY;
        double distanceToApproachTarget = Math.hypot(approachDx, approachDy);

        double drivePower = 0.0;
        double turnPower = 0.0;
        boolean finalHeadingMode = false;

        if (!positionReached) {
            // ---------- Phase 1: go to the point ----------
            // Latch this transition. Rotation for the final heading can move
            // the measured pose slightly outside the tolerance, but it must
            // not restart point-chasing and abandon the final heading.
            if (distanceToApproachTarget <= DISTANCE_TOLERANCE_MM) {
                positionReached = true;
                stopDrive();
                return;
            }

            double targetHeadingToPoint = Math.toDegrees(
                    Math.atan2(approachDy, approachDx)
            );
            double headingError = normalizeAngle(targetHeadingToPoint - currentHeadingDeg);

            // Fold into headless line (-90..90). Prefer driving backward over spinning 180°.
            double driveSign = 1.0;
            if (headingError > 90.0) {
                headingError -= 180.0;
                driveSign = -1.0;
            } else if (headingError < -90.0) {
                headingError += 180.0;
                driveSign = -1.0;
            }

            if (Math.abs(headingError) > HEADING_LOCK_DEG) {
                // still too far off-axis → rotate in place, no translation
                drivePower = 0.0;
                turnPower = KP_TURN * headingError;
            } else {
                // aligned enough (forward OR backward) → translate + gentle heading correction
                drivePower = driveSign * KP_DRIVE * distanceToApproachTarget;
                turnPower  = KP_TURN * headingError * 0.5;
            }
        } else {
            // ---------- Phase 2: orient to final heading ----------
            finalHeadingMode = true;
            double headingError = normalizeAngle(targetHeadingDeg - currentHeadingDeg);

            // Final-heading rotation can shift the robot slightly. Only finish
            // when it is still at the target point as well as at the requested
            // heading; otherwise reacquire the point before trying again.
            if (Math.abs(headingError) <= HEADING_TOLERANCE_DEG) {
                if (distanceToTarget <= DISTANCE_TOLERANCE_MM) {
                    terminat = true;
                    stopDrive();
                    return;
                }

                // The COR estimate was not accurate enough. Recompute a new
                // compensated approach point from this heading and try again.
                positionReached = false;
                stopDrive();
                return;
            }

            // pure rotation
            drivePower = 0.0;
            turnPower = KP_TURN * headingError;
        }

        // ----- Power limiting & deadband -----
        drivePower = clamp(drivePower, -MAX_POWER, MAX_POWER);
        turnPower  = clamp(turnPower,  -MAX_POWER, MAX_POWER);

        if (drivePower != 0 && Math.abs(drivePower) < MIN_DRIVE_POWER) {
            drivePower = Math.copySign(MIN_DRIVE_POWER, drivePower);
        }
        double minimumTurnPower = finalHeadingMode
                ? FINAL_MIN_TURN_POWER
                : MIN_TURN_POWER;
        if (turnPower != 0 && Math.abs(turnPower) < minimumTurnPower) {
            turnPower = Math.copySign(minimumTurnPower, turnPower);
        }

        double leftPower  = drivePower - turnPower;
        double rightPower = drivePower + turnPower;

        // keep both motors inside [-1, 1]
        double maxMag = Math.max(1.0, Math.max(Math.abs(leftPower), Math.abs(rightPower)));
        leftPower  /= maxMag;
        rightPower /= maxMag;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }
    private void stopDrive() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    private double normalizeAngle(double angleDeg) {
        while (angleDeg > 180) angleDeg -= 360;
        while (angleDeg < -180) angleDeg += 360;
        return angleDeg;
    }

    private double[] fieldOffsetAtHeading(double headingDeg) {
        double headingRad = Math.toRadians(headingDeg);
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);

        return new double[] {
                COR_FORWARD_MM * cos - COR_LEFT_MM * sin,
                COR_FORWARD_MM * sin + COR_LEFT_MM * cos
        };
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
