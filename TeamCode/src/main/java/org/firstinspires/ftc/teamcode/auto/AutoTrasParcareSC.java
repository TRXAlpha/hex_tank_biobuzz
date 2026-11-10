
package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Calibrare deplasare fata-spate (axa X) - tank drive + goBILDA Pinpoint V2.
 *
 * Robotul merge 1000 mm inainte si 1000 mm inapoi, de 5 ori.
 * Distanta se masoara cu Pinpoint (pod X, 4-bar, roti omni 32 mm).
 *
 * CONFIGURARE in Driver Station (Robot Configuration):
 *   - motor port 0 -> "left_drive"
 *   - motor port 1 -> "right_drive"
 *   - I2C Pinpoint  -> "odo"
 *
 * NECESITA fisierul GoBildaPinpointDriver.java in folderul teamcode
 * (descarcat de pe goBILDA). Metodele de mai jos sunt cele din versiunea noua
 * (cu DistanceUnit / AngleUnit). Daca driverul tau e mai vechi, getPosX()/getHeading()
 * nu primesc parametri.
 *
 * MOD DE CALIBRARE:
 *   1. Pune robotul pe o linie/banda de masurat, marcheaza pozitia rotii/botului.
 *   2. Ruleaza. La capatul fiecarui drum INAINTE robotul se opreste si asteapta (A).
 *   3. Masoara cu ruleta distanta reala parcursa fata de start (in mm).
 *   4. Pinpoint raporteaza ~1000 mm. Factor de corectie:
 *        noua_rezolutie = REZOLUTIE_CURENTA * (distanta_raportata / distanta_reala)
 *      Ex: raporteaza 1000, real 980 -> 19.894 * 1000/980 = 20.30 ticks/mm
 *   5. Pune valoarea noua in ENCODER_TICKS_PER_MM si repeta pana distanta reala ~ 1000 mm.
 *   Daca robotul se intoarce in alt loc decat startul, ai o eroare de offset/drift.
 */
@Autonomous(name = "auto tras parcare SC")
@Config
public class AutoTrasParcareSC extends LinearOpMode {

    // ======================= PARAMETRI DE REGLAT =======================

    // --- Traseu ---
    public static double DISTANTA_MM = 1000.0;
    public static int NR_REPETARI = 5;

    // --- Pinpoint ---
    // Rezolutie pod goBILDA 4-bar: ~19.894 ticks/mm (valoare implicita).
    // Modifica dupa calibrare (vezi pasul 4 de mai sus).

    // Offset-urile podurilor fata de punctul de urmarire al robotului (mm).
    // Punctul de urmarire = centrul axei rotilor motoare (mijlocul celor 330 mm).
    // X pod offset: cat de la stanga (+) / dreapta (-) fata de centru e podul X
    // Y pod offset: cat in fata (+) / in spate (-) fata de centru e podul Y
    // Pentru deplasarea in linie dreapta nu conteaza mult, dar completeaza-le corect.
    private static final double X_POD_OFFSET_MM = -44.3;
    private static final double Y_POD_OFFSET_MM = -6.6;

    // Daca podul X da valori NEGATIVE cand robotul merge inainte -> schimba in REVERSED
    private static final GoBildaPinpointDriver.EncoderDirection X_DIR =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;
    private static final GoBildaPinpointDriver.EncoderDirection Y_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // --- Motoare ---
    // Daca robotul merge inapoi cand ar trebui inainte, inverseaza ambele.
    private static final DcMotorSimple.Direction stanga = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction dreapta = DcMotorSimple.Direction.FORWARD;

    // --- Control ---
    public static  double MAX_POWER = 0.5;     // viteza maxima
    public static  double MIN_POWER = 0.12;    // sa nu se blocheze aproape de tinta
    public static  double KP_DIST = 0.003;     // putere per mm eroare
    public static  double KP_HEADING = 1.5;    // corectie de directie (putere per radian)
    private static final double TOLERANTA_MM = 3.0;  // cat de aproape = "ajuns"
    private static final double TIMEOUT_S = 10.0;    // siguranta per deplasare

    // Apasa A la capat de drum inainte pentru a masura manual cu ruleta
    private static final boolean ASTEAPTA_MASURARE = true;

    // ===================================================================

    private DcMotor left, right;
    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {
        left = hardwareMap.get(DcMotor.class, "stanga");
        right = hardwareMap.get(DcMotor.class, "dreapta");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        left.setDirection(stanga);
        right.setDirection(dreapta);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odo.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(X_DIR, Y_DIR);
        odo.resetPosAndIMU();   // robotul trebuie sa stea nemiscat ~0.25 s

        telemetry.addLine("Pune robotul la start si NU-L misca.");
        telemetry.addLine("Apasa START cand e gata.");
        telemetry.update();

        waitForStart();





        if (isStopRequested()) return;

        // Reset pozitie: 0,0 si heading 0 la start
        odo.resetPosAndIMU();
        sleep(400);
        odo.update();

        double startX = odo.getPosX(DistanceUnit.MM);
        double startHeading = odo.getHeading(AngleUnit.RADIANS);

        for (int i = 4; i <= NR_REPETARI && opModeIsActive(); i++) {

            // ---- INAINTE ----
            double raportat = driveTo(startX + DISTANTA_MM, startHeading,
                    "Rep " + i + "/" + NR_REPETARI + " INAINTE");
            if (!opModeIsActive()) break;

            if (ASTEAPTA_MASURARE) {
                while (opModeIsActive() && !gamepad1.a) {
                    odo.update();
                    telemetry.addData("Repetare", "%d / %d", i, NR_REPETARI);
                    telemetry.addData("Pinpoint X (mm)", "%.1f", odo.getPosX(DistanceUnit.MM) - startX);
                    telemetry.addData("Pinpoint Y (mm)", "%.1f", odo.getPosY(DistanceUnit.MM));
                    telemetry.addData("Heading (grade)", "%.2f", Math.toDegrees(
                            odo.getHeading(AngleUnit.RADIANS) - startHeading));
                    telemetry.addLine("Masoara distanta REALA cu ruleta.");
                    telemetry.addLine("Apasa A pentru a continua (inapoi).");
                    telemetry.update();
                    sleep(20);
                }
                while (opModeIsActive() && gamepad1.a) sleep(20); // asteapta eliberarea
            } else {
                sleep(500);
            }

            // ---- INAPOI ----
            driveTo(startX, startHeading, "Rep " + i + "/" + NR_REPETARI + " INAPOI");
            sleep(500);
        }

        stopMotors();

        // Rezultat final
        while (opModeIsActive()) {
            odo.update();
            telemetry.addLine("FINALIZAT");
            telemetry.addData("Eroare finala X (mm)", "%.1f", odo.getPosX(DistanceUnit.MM) - startX);
            telemetry.addData("Eroare finala Y (mm)", "%.1f", odo.getPosY(DistanceUnit.MM));
            telemetry.addData("Eroare heading (grade)", "%.2f", Math.toDegrees(
                    odo.getHeading(AngleUnit.RADIANS) - startHeading));
            telemetry.addLine("Masoara cat de departe e robotul de punctul de start.");
            telemetry.update();
            sleep(50);
        }
    }

    /**
     * Merge pe axa X pana la pozitia tinta (mm), mentinand heading-ul.
     * Returneaza pozitia X finala raportata de Pinpoint.
     */
    private double driveTo(double targetXmm, double targetHeading, String eticheta) {
        ElapsedTime timeout = new ElapsedTime();
        ElapsedTime stabil = new ElapsedTime();
        boolean inTolerance = false;

        while (opModeIsActive() && timeout.seconds() < TIMEOUT_S) {
            odo.update();

            double x = odo.getPosX(DistanceUnit.MM);
            double heading = odo.getHeading(AngleUnit.RADIANS);

            double eroare = targetXmm - x;

            // Putere proportionala, cu minim ca sa ajunga la tinta
            double p = KP_DIST * eroare;
            double semn = Math.signum(p);
            p = Math.min(Math.abs(p), MAX_POWER);
            p = Math.max(p, MIN_POWER) * semn;

            // Corectie de directie (heading wrap la [-pi, pi])
            double eroareHeading = wrap(targetHeading - heading);
            double turn = KP_HEADING * eroareHeading;
            // la mers inapoi corectia se inverseaza


            double pStanga, pDreapta;
            if (Math.abs(eroare) <= TOLERANTA_MM) {
                pStanga = 0;
                pDreapta = 0;
                if (!inTolerance) {
                    inTolerance = true;
                    stabil.reset();
                }
                if (stabil.seconds() > 0.3) break; // stabil 300 ms in toleranta
            } else {
                inTolerance = false;
                pStanga = clip(p - turn);
                pDreapta = clip(p + turn);
            }

            left.setPower(pStanga);
            right.setPower(pDreapta);

            telemetry.addLine(eticheta);
            telemetry.addData("Tinta X (mm)", "%.1f", targetXmm);
            telemetry.addData("Pinpoint X (mm)", "%.1f", x);
            telemetry.addData("Eroare (mm)", "%.1f", eroare);
            telemetry.addData("Heading err (grade)", "%.2f", Math.toDegrees(eroareHeading));
            telemetry.addData("Putere L/R", "%.2f / %.2f", pStanga, pDreapta);
            telemetry.update();
        }

        stopMotors();
        sleep(300);
        odo.update();
        return odo.getPosX(DistanceUnit.MM);
    }

    private void stopMotors() {
        left.setPower(0);
        right.setPower(0);
    }

    private static double clip(double v) {
        return Math.max(-1.0, Math.min(1.0, v));
    }

    private static double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
//left/right motoarele, stanga dreapta directia