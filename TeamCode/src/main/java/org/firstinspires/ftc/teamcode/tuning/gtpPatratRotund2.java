package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Deplaseaza robotul (tank drive, 2 motoare: stanga/dreapta) pe un traseu patrat cu
 * latura de 600 mm, pornind si revenind in (0,0), folosind pozitia raportata de
 * Pinpoint Odometry Computer. Traseul e parcurs colt cu colt, folosind aceeasi
 * metoda goToPoint() pentru fiecare segment.
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
@Autonomous(name = "Patrat 600mm 2", group = "Auto")
public class gtpPatratRotund2 extends LinearOpMode {

    // ----- Hardware -----
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private GoBildaPinpointDriver pinpoint;

    // ----- Traseu: patrat cu latura de 600 mm, pornind din (0,0) -----
    // Ordinea: (0,0) -> (600,0) -> (600,600) -> (0,600) -> (0,0)
    public static double SIDE_MM = 600.0;

    static final double[][] WAYPOINTS = {
            {SIDE_MM, 0.0},
            {SIDE_MM, SIDE_MM},
            {0.0,     SIDE_MM},
            {0.0,     0.0}
    };

    // ----- Tolerante oprire -----
    public static double DISTANCE_TOLERANCE_MM = 15.0;

    public static double HEADING_TOLERANCE_FINAL = 10.0;
    public static double HEADING_TOLERANCE_DEG = 5.0;

    // ----- Coeficienti proportionali (de reglat empiric pe robotul vostru) -----
    public static double KP_TURN         = 0.010;  // putere motor per grad eroare unghi
    public static double KP_DRIVE        = 0.0035; // putere motor per mm distanta ramasa
    public static double MAX_POWER       = 0.7;
    public static double MIN_TURN_POWER  = 0.15 ;   // putere minima ca sa invinga frecarea la rotire
    public static double MIN_DRIVE_POWER = 0.2;   // putere minima ca sa invinga frecarea la deplasare

    // Raza (mm) la care controller-ul considera un colt intermediar "atins" si trece
    // la urmatorul punct FARA sa opreasca robotul -> asta produce curba lina in colt,
    // in loc de oprire + rotire pe loc. Cu cat e mai mare, cu atat colturile sunt mai
    // "taiate" (curba mai larga); cu cat e mai mica, cu atat traseul e mai aproape de
    // un patrat perfect dar cu tranzitii mai putin line. De reglat empiric.
    public static double CORNER_RADIUS_MM = 120.0;

    boolean ignoreXY = false;

    @Override
    public void runOpMode() {
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


        telemetry.addLine("Calibrare IMU Pinpoint, robotul trebuie sa stea nemiscat...");
        telemetry.update();
        sleep(500); // timp pentru calibrarea IMU-ului intern al Pinpoint-ului

        telemetry.addLine("Gata. Apasa START pentru a merge pe traseul patrat");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Se reseteaza pozitia chiar inainte de start -> (0,0) este punctul curent
        pinpoint.resetPosAndIMU();
        sleep(300);

        for (int i = 0; i < WAYPOINTS.length && opModeIsActive(); i++) {
            double wx = WAYPOINTS[i][0];
            double wy = WAYPOINTS[i][1];
            boolean isLastPoint = (i == WAYPOINTS.length - 1);

            // Coltul final se atinge precis si robotul se opreste complet.
            // Colturile intermediare se "ating" cu o toleranta mai mare, iar
            // robotul NU se opreste - trece direct spre urmatorul punct -> tranzitie lina.
            double toleranceMm = isLastPoint ? DISTANCE_TOLERANCE_MM : CORNER_RADIUS_MM;

            telemetry.addData("Colt", (i + 1) + " / " + WAYPOINTS.length);
            telemetry.addData("Tinta (mm)", "%.0f, %.0f", wx, wy);
            telemetry.update();

            goToPoint(wx, wy, toleranceMm, isLastPoint);
        }

        telemetry.addLine("Traseu patrat finalizat - robotul a revenit la (0,0)");
        telemetry.update();
        sleep(1500);
    }

    /**
     * Deplaseaza robotul (fara strafe - tank drive) catre (targetX, targetY) in mm,
     * folosind feedback de la Pinpoint.
     *
     * Control CONTINUU (nu mai exista faza separata de "rotire pe loc, apoi deplasare"):
     * puterea de mers inainte e scalata cu cosinusul erorii de unghi, deci robotul
     * incetineste natural cand trebuie sa vireze strans, dar nu se opreste niciodata
     * complet doar ca sa se roteasca - asta produce o miscare curbata, lina.
     *
     * @param toleranceMm      raza (mm) la care se considera punctul "atins"
     * @param isLastPoint daca true, motoarele sunt oprite complet la sfarsit
     *                            (folosit pentru ultimul punct din traseu); daca false,
     *                            metoda doar returneaza control, fara sa franeze,
     *                            astfel incat urmatorul segment continua lin.
     */
    private void goToPoint(double targetX, double targetY, double toleranceMm, boolean isLastPoint) {

        while (opModeIsActive()) {

            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            double currentX = pose.getX(DistanceUnit.MM);
            double currentY = pose.getY(DistanceUnit.MM);
            double currentHeadingDeg = pose.getHeading(AngleUnit.DEGREES);

            double dx = targetX - currentX;
            double dy = targetY - currentY;
            double distanceToTarget = Math.hypot(dx, dy);

            telemetry.addData("distanceToTarget:",distanceToTarget);
            telemetry.update();

            if (distanceToTarget <= toleranceMm && !isLastPoint) {
                break; // punct atins
            }

            double targetHeadingDeg = Math.toDegrees(Math.atan2(dy, dx));
            if(ignoreXY){
                targetHeadingDeg = 0;
            }
            double headingError = normalizeAngle(targetHeadingDeg - currentHeadingDeg);

            if(distanceToTarget <= toleranceMm && isLastPoint && Math.abs(headingError) <= HEADING_TOLERANCE_FINAL) {
                break;
            }

            if(distanceToTarget <= toleranceMm && isLastPoint) {
                ignoreXY = true;
            }


            // headingScale = 1 cand robotul e aliniat perfect cu directia tintei,
            // scade lin spre 0 pe masura ce eroarea de unghi creste spre 90 grade,
            // si devine 0 (nu mai merge inainte deloc) peste 90 grade eroare.
            double headingErrorRad = Math.toRadians(headingError);
            double headingScale = Math.max(0.0, Math.cos(headingErrorRad));

            double turnPower  = KP_TURN * headingError;
            double drivePower = KP_DRIVE * distanceToTarget * headingScale;

            drivePower = clamp(drivePower, -MAX_POWER, MAX_POWER);
            turnPower  = clamp(turnPower,  -MAX_POWER, MAX_POWER);

            if (drivePower != 0 && Math.abs(drivePower) < MIN_DRIVE_POWER && headingScale > 0.05) {
                drivePower = Math.copySign(MIN_DRIVE_POWER, drivePower);
            }
            if (Math.abs(headingError) > HEADING_TOLERANCE_DEG
                    && turnPower != 0 && Math.abs(turnPower) < MIN_TURN_POWER) {
                turnPower = Math.copySign(MIN_TURN_POWER, turnPower);
            }

            if(ignoreXY){
                drivePower = 0;
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
            //telemetry.update();
        }

        if (isLastPoint) {
            stopDrive();
        }
    }

    private void stopDrive() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    private double normalizeAngle(double angleDeg) {
        while (angleDeg > 180)  angleDeg -= 360;
        while (angleDeg < -180) angleDeg += 360;
        return angleDeg;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}