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
@Autonomous(name = "GoTo 600,600 mm", group = "Auto")
public class goToPoint extends OpMode {

    // ----- Hardware -----
    DcMotor leftDrive, rightDrive;
    private GoBildaPinpointDriver pinpoint;

    // ----- Punct tinta -----
    public static double TARGET_X_MM = 600.0;
    public static double TARGET_Y_MM = 600.0;

    // ----- Tolerante oprire -----
    public static double DISTANCE_TOLERANCE_MM = 15.0;
    static final double HEADING_TOLERANCE_DEG = 2.0;

    // ----- Coeficienti proportionali (de reglat empiric pe robotul vostru) -----
    public static double KP_TURN         = 0.020;  // putere motor per grad eroare unghi
    public static double KP_DRIVE        = 0.0035; // putere motor per mm distanta ramasa
    public static double MAX_POWER       = 0.30;
    public static double MIN_TURN_POWER  = 0.12;   // putere minima ca sa invinga frecarea la rotire
    public static double MIN_DRIVE_POWER = 0.15;   // putere minima ca sa invinga frecarea la deplasare
    public static double HEADING_LOCK_DEG = 10.0;  // sub aceasta eroare de unghi incepe sa mearga inainte
    boolean terminat = false;
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

    }


    @Override
    public void loop(){
        goToPoint(TARGET_X_MM, TARGET_Y_MM);

        if(terminat) {
            stopDrive();
            telemetry.addData("terminat", terminat);
            telemetry.update();
        }

    }

     void goToPoint(double targetX, double targetY) {

            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            // x,y,h
            double currentX = pose.getX(DistanceUnit.MM);
            double currentY = pose.getY(DistanceUnit.MM);
            double currentHeadingDeg = pose.getHeading(AngleUnit.DEGREES);
            // delte si dist
            double dx = targetX - currentX;
            double dy = targetY - currentY;
            double distanceToTarget = Math.hypot(dx, dy);

            if (distanceToTarget <= DISTANCE_TOLERANCE_MM){
                terminat = true;
                return; // am ajuns

            }

            // heading
            double targetHeadingDeg = Math.toDegrees(Math.atan2(dy, dx));
            double headingError = normalizeAngle(targetHeadingDeg - currentHeadingDeg);

            double drivePower;
            double turnPower;

            if (Math.abs(headingError) > HEADING_LOCK_DEG) {
                // eroare mare de unghi -> ne rotim pe loc, fara sa mergem inainte
                drivePower = 0.0;
                turnPower = KP_TURN * headingError;
            } else {
                // mergem inainte si corectam usor directia
                drivePower = KP_DRIVE * distanceToTarget;
                turnPower  = KP_TURN * headingError * 0.5; // corectie mai blanda in miscare
            }

            drivePower = clamp(drivePower, -MAX_POWER, MAX_POWER);
            turnPower  = clamp(turnPower, -MAX_POWER, MAX_POWER);

            // ks
            if (drivePower != 0 && Math.abs(drivePower) < MIN_DRIVE_POWER) {
                drivePower = Math.copySign(MIN_DRIVE_POWER, drivePower);
            }
            if (turnPower != 0 && Math.abs(turnPower) < MIN_TURN_POWER) {
                turnPower = Math.copySign(MIN_TURN_POWER, turnPower);
            }

            double leftPower  = drivePower - turnPower;
            double rightPower = drivePower + turnPower;

            double maxMag = Math.max(1.0, Math.max(Math.abs(leftPower), Math.abs(rightPower)));
            leftPower  /= maxMag;
            rightPower /= maxMag;

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            telemetry.addData("X (mm)", "%.1f", currentX);
            telemetry.addData("Y (mm)", "%.1f", currentY);
            telemetry.addData("Heading (deg)", "%.1f", currentHeadingDeg);
            telemetry.addData("Distanta ramasa (mm)", "%.1f", distanceToTarget);
            telemetry.addData("Eroare unghi (deg)", "%.1f", headingError);
            telemetry.update();

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

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
