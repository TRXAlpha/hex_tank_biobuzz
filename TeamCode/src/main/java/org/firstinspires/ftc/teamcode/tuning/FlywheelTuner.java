package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Flywheel;

/*
 * FLYWHEEL TUNER - the screen tells you what to do. Everything saves by itself.
 *
 * STEP 1 (once, and again after you change the shooter):
 *   Take the balls out. Press TRIANGLE. Wait ~15 s. Done.
 *
 * STEP 2 (build the distance table):
 *   Put the robot on the field. Measure with a tape: turret center -> goal wall
 *   (floor, under the AprilTags). Set that distance with D-pad LEFT / RIGHT.
 *   CROSS = flywheel on.  Hold R2 = shoot.
 *   Ball too short -> D-pad UP.   Ball too long -> D-pad DOWN.   (L1 / R1 = big steps)
 *   Ball goes in 3 times in a row -> press CIRCLE (saves this distance).
 *   Move the robot ~30 cm and repeat. Get at least 5 distances, near to far.
 *
 * Other:  hold SQUARE = unjam (intake + stagiu reverse).  SHARE = delete point at this distance.
 */
@Config
@TeleOp(name = "Flywheel Tuner", group = "tuning")
public class FlywheelTuner extends OpMode {

    // calibration sweep: powers tried, time to settle, time to measure
    private static final double[] CAL_POWERS = {0.3, 0.45, 0.6, 0.75, 0.9};
    private static final double CAL_SETTLE_S = 2.0, CAL_MEASURE_S = 1.0;

    private Flywheel fw;
    private DcMotor stagiu, intake;

    private boolean fileFound, calibratedNow;
    public static double distCm = 150, speed = 1120;
    private String msg = "";

    // calibration state
    private boolean cal = false;
    private int calStep;
    private final ElapsedTime calTimer = new ElapsedTime();
    private final double[] calE = new double[CAL_POWERS.length], calV = new double[CAL_POWERS.length];
    private double sumV, sumVolts;
    private int nSamples;

    // shot counter
    private boolean wasFeeding;
    private int shots;
    private double lastShotPct;

    // buttons (edge detect)
    private boolean pTri, pCross, pCircle, pShare, pUp, pDown, pLeft, pRight, pL1, pR1;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        fileFound = Flywheel.load();
        fw = new Flywheel(hardwareMap);

        stagiu = hardwareMap.dcMotor.get("stagiu");
        stagiu.setDirection(DcMotorSimple.Direction.REVERSE);
        stagiu.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake = hardwareMap.dcMotor.get("intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (!Flywheel.TABLE.isEmpty()) speed = Flywheel.speedForDistance(distCm);
    }

    @Override
    public void init_loop() {
        telemetry.addLine(fileFound ? "Saved values found. Press START."
                                    : "No saved values yet. Press START, then do STEP 1.");
        telemetry.addData("Battery", "%.2f V %s", fw.getVolts(), fw.getVolts() < 12.0 ? " <- CHARGE IT" : "");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (pressed(gamepad1.triangle, pTri)) {
            if (cal) cancelCal("Calibration cancelled.");
            else startCal();
        }
        pTri = gamepad1.triangle;

        if (cal) {
            calLoop();
            saveButtons();
            return;
        }

        // ---------- buttons ----------
        if (pressed(gamepad1.cross, pCross)) fw.setOn(!fw.isOn());
        if (pressed(gamepad1.dpad_up, pUp))       speed += 5;
        if (pressed(gamepad1.dpad_down, pDown))   speed -= 5;
        if (pressed(gamepad1.right_bumper, pR1))  speed += 100;
        if (pressed(gamepad1.left_bumper, pL1))   speed -= 100;
        if (pressed(gamepad1.dpad_right, pRight)) changeDist(+10);
        if (pressed(gamepad1.dpad_left, pLeft))   changeDist(-10);
        if (pressed(gamepad1.circle, pCircle)) {
            Flywheel.putPoint(distCm, speed);
            Flywheel.save();
            msg = String.format("SAVED %.0f cm = %.0f. Now move ~30 cm.", distCm, speed);
        }
        if (pressed(gamepad1.share, pShare)) {
            msg = Flywheel.removePoint(distCm) ? "Deleted point at " + (int) distCm + " cm." : "No point here.";
            Flywheel.save();
        }
        saveButtons();
        speed = Math.max(0, speed);

        // ---------- flywheel ----------
        fw.setTarget(speed);
        fw.update();

        // ---------- feed: hold R2, only when at speed ----------
        boolean feeding = gamepad1.right_trigger > 0.5 && fw.isOn() && fw.isAtSpeed();
        if (gamepad1.square) {
            stagiu.setPower(-1);
            intake.setPower(-1);
        } else if (feeding) {
            stagiu.setPower(1);
            intake.setPower(1);
        } else {
            stagiu.setPower(0);
            intake.setPower(0);
        }
        if (feeding && !wasFeeding) {
            shots++;
            lastShotPct = fw.getVelocity() / speed * 100;
        }
        wasFeeding = feeding;

        // ---------- screen ----------
        boolean ready = fileFound || calibratedNow;
        if (!ready) {
            telemetry.addLine(">>> STEP 1: take balls out, press TRIANGLE, wait 15 s <<<");
        } else {
            telemetry.addLine(">>> STEP 2: tape-measure turret -> goal. Set distance (D-pad L/R).");
            telemetry.addLine("CROSS = flywheel on. Hold R2 = shoot.");
            telemetry.addLine("Short -> D-pad UP. Long -> D-pad DOWN. In 3x -> CIRCLE.");
        }
        if (!msg.isEmpty()) telemetry.addLine(msg);
        telemetry.addLine("");
        telemetry.addData("DISTANCE", "%.0f cm", distCm);
        telemetry.addData("SPEED", "%.0f  (flywheel %s)", speed, fw.isOn() ? "ON" : "off");
        telemetry.addData("READY TO SHOOT", fw.isAtSpeed() ? "YES" : "no");
        if (fw.isOn() && speed > fw.maxSpeedNow()) {
            telemetry.addLine("!! TOO FAST for this battery: max ~" + (int) fw.maxSpeedNow() + " !!");
        }
        telemetry.addData("Battery", "%.2f V %s", fw.getVolts(), fw.getVolts() < 11.5 ? " <- LOW, CHANGE IT" : "");
        telemetry.addData("Shots / last at", "%d / %.1f %%", shots, lastShotPct);
        telemetry.addData("Saved table", Flywheel.tableString());
        telemetry.addLine("");
        telemetry.addData("vel", fw.getVelocity());       // graph these two on Dashboard
        telemetry.addData("target", speed);
        telemetry.addData("power", "%.2f", fw.getPower());
        double vd = fw.getVelD(), vs = fw.getVelS();
        if (fw.isOn() && Math.abs(vd - vs) > 0.1 * Math.max(Math.abs(vd), Math.abs(vs))) {
            telemetry.addLine("!! LEFT/RIGHT speeds differ >10% - check encoder cables / belts !!");
        }
        telemetry.addData("vel D / S", "%.0f / %.0f", vd, vs);
        telemetry.addData("KS / KV / KP", "%.3f / %.6f / %.4f", Flywheel.KS, Flywheel.KV, Flywheel.KP);
        telemetry.update();
    }

    // ===================== calibration =====================

    private void startCal() {
        cal = true;
        calStep = 0;
        resetSamples();
        calTimer.reset();
        fw.setOn(false);
        msg = "";
    }

    private void cancelCal(String why) {
        cal = false;
        fw.stop();
        msg = why;
    }

    private void calLoop() {
        stagiu.setPower(0);
        intake.setPower(0);
        double p = CAL_POWERS[calStep];
        fw.setRawPower(p);

        double t = calTimer.seconds();
        if (t > CAL_SETTLE_S) {
            sumV += fw.getVelocity();
            sumVolts += fw.getVolts();
            nSamples++;
        }
        if (t > CAL_SETTLE_S + CAL_MEASURE_S) {
            calV[calStep] = sumV / nSamples;
            calE[calStep] = p * (sumVolts / nSamples) / Flywheel.NOMINAL_V;   // power as if 12 V
            calStep++;
            resetSamples();
            calTimer.reset();
            if (calStep == CAL_POWERS.length) {
                finishCal();
                return;
            }
        }

        telemetry.addLine("CALIBRATING - do not touch. TRIANGLE = cancel.");
        telemetry.addData("Step", "%d / %d  (power %.2f)", calStep + 1, CAL_POWERS.length, p);
        telemetry.addData("Speed now", "%.0f", fw.getVelocity());
        telemetry.addData("Battery", "%.2f V", fw.getVolts());
        telemetry.update();
    }

    /** Straight line through (speed, 12V-power): power = KS + KV * speed. */
    private void finishCal() {
        int n = CAL_POWERS.length;
        double mv = 0, me = 0;
        for (int i = 0; i < n; i++) { mv += calV[i]; me += calE[i]; }
        mv /= n; me /= n;
        double num = 0, den = 0;
        for (int i = 0; i < n; i++) {
            num += (calV[i] - mv) * (calE[i] - me);
            den += (calV[i] - mv) * (calV[i] - mv);
        }
        double kv = den > 0 ? num / den : 0;
        double ks = me - kv * mv;

        if (kv <= 0 || ks < -0.05 || ks > 0.3) {
            cancelCal(String.format("CALIBRATION FAILED (KS %.3f KV %.6f). Flywheel stuck or encoder unplugged?", ks, kv));
            return;
        }
        Flywheel.KS = Math.max(0, ks);
        Flywheel.KV = kv;
        Flywheel.save();
        calibratedNow = true;
        cancelCal(String.format("CALIBRATION OK. Max speed at 12 V ~%.0f. Now do STEP 2.", (1 - ks) / kv));
    }

    private void resetSamples() {
        sumV = 0;
        sumVolts = 0;
        nSamples = 0;
    }

    // ===================== helpers =====================

    private void changeDist(double d) {
        distCm = Math.max(0, distCm + d);
        if (!Flywheel.TABLE.isEmpty()) speed = Math.round(Flywheel.speedForDistance(distCm));   // good start guess
    }

    private static boolean pressed(boolean now, boolean before) {
        return now && !before;
    }

    private void saveButtons() {
        pCross = gamepad1.cross; pCircle = gamepad1.circle; pShare = gamepad1.share;
        pUp = gamepad1.dpad_up; pDown = gamepad1.dpad_down;
        pLeft = gamepad1.dpad_left; pRight = gamepad1.dpad_right;
        pL1 = gamepad1.left_bumper; pR1 = gamepad1.right_bumper;
    }

    @Override
    public void stop() {
        fw.stop();
        stagiu.setPower(0);
        intake.setPower(0);
    }
}
