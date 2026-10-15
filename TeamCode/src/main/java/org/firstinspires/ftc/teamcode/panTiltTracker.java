package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@Config
@TeleOp(name = "LL Pan-Tilt Tracker")
public class panTiltTracker extends LinearOpMode {

    // ================= TUNING =================
    // goBILDA Dual Mode in servo mode = 300 deg over 500-2500 us
    public static double SERVO_RANGE_DEG = 300.0;

    // Flip these if an axis runs AWAY from the tag
    public static double PAN_SIGN  = 1.0;
    public static double TILT_SIGN = -1.0;

    // Fraction of the estimated error corrected per camera frame.
    // 1.0 = snap, 0.6 = fast + smooth, 0.3 = silky/slow
    public static double K_PAN  = 0.6;
    public static double K_TILT = 0.6;

    public static double DEADBAND_DEG   = 0.4;   // ignore tx/ty noise below this
    public static double MAX_RATE_DEG_S = 400.0; // max servo slew, limits jerk

    // Mechanical limits (deg from center) - SET THESE FOR YOUR CABLES/FRAME
    public static double PAN_MIN = -90, PAN_MAX = 90;
    public static double TILT_MIN = -20, TILT_MAX = 45;

    // Servo position (0..1) that points the camera straight ahead / level
    public static double PAN_CENTER  = 0.5;
    public static double TILT_CENTER = 0.5;

    // USB + loop + servo travel delay not reported by the Limelight
    public static double EXTRA_LATENCY_MS = 20.0;

    // Lost-tag behavior: hold, then drift back to center
    public static double LOST_HOLD_MS   = 500.0;
    public static double RETURN_RATE_DEG_S = 60.0;
    // ==========================================

    // History of commanded angles, used for latency compensation
    static final int HIST = 256;
    final double[] hT = new double[HIST], hPan = new double[HIST], hTilt = new double[HIST];
    int hIdx = 0, hCount = 0;

    void record(double t, double pan, double tilt) {
        hT[hIdx] = t; hPan[hIdx] = pan; hTilt[hIdx] = tilt;
        hIdx = (hIdx + 1) % HIST;
        if (hCount < HIST) hCount++;
    }

    // Where the camera was pointing at time tq
    double[] lookup(double tq) {
        int best = -1;
        for (int i = 0; i < hCount; i++) {
            int j = (hIdx - 1 - i + HIST) % HIST;
            best = j;
            if (hT[j] <= tq) break;
        }
        return best < 0 ? new double[]{0, 0} : new double[]{hPan[best], hTilt[best]};
    }

    static double step(double from, double to, double maxStep) {
        return from + Range.clip(to - from, -maxStep, maxStep);
    }

    FtcDashboard dashboard;
    @Override
    public void runOpMode() {
        ServoImplEx pan  = hardwareMap.get(ServoImplEx.class, "pan");
        ServoImplEx tilt = hardwareMap.get(ServoImplEx.class, "tilt");
        pan.setPwmRange(new PwmControl.PwmRange(500, 2500));
        tilt.setPwmRange(new PwmControl.PwmRange(500, 2500));

        dashboard = FtcDashboard.getInstance();
        Limelight3A ll = hardwareMap.get(Limelight3A.class, "limelight");
        ll.setPollRateHz(100);
        ll.pipelineSwitch(2);   // your AprilTag pipeline
        ll.start();

        dashboard.startCameraStream(ll, 30);

        double panCmd = 0, tiltCmd = 0;    // degrees, 0 = center
        double panGoal = 0, tiltGoal = 0;
        double lastStale = Double.MAX_VALUE;
        double lastSeen = -1e9;

        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        double lastLoop = timer.milliseconds();

        while (opModeIsActive()) {
            double now = timer.milliseconds();
            double dt = (now - lastLoop) / 1000.0;
            lastLoop = now;

            LLResult r = ll.getLatestResult();
            boolean fresh = false;
            if (r != null) {
                double stale = r.getStaleness();
                fresh = stale < lastStale;   // staleness resets on each new frame
                lastStale = stale;

                if (fresh && r.isValid()) {
                    double agoMs = stale + r.getCaptureLatency()
                            + r.getTargetingLatency() + EXTRA_LATENCY_MS;
                    double[] past = lookup(now - agoMs);

                    // Absolute angle of the tag = where we pointed then + offset seen
                    double panTarget  = past[0] + PAN_SIGN  * r.getTx();
                    double tiltTarget = past[1] + TILT_SIGN * r.getTy();

                    double ePan = panTarget - panGoal;
                    double eTilt = tiltTarget - tiltGoal;
                    if (Math.abs(ePan)  > DEADBAND_DEG) panGoal  += K_PAN  * ePan;
                    if (Math.abs(eTilt) > DEADBAND_DEG) tiltGoal += K_TILT * eTilt;

                    lastSeen = now;
                }
            }

            if (now - lastSeen > LOST_HOLD_MS) {
                double s = RETURN_RATE_DEG_S * dt;
                panGoal = step(panGoal, 0, s);
                tiltGoal = step(tiltGoal, 0, s);
            }

            panGoal  = Range.clip(panGoal,  PAN_MIN,  PAN_MAX);
            tiltGoal = Range.clip(tiltGoal, TILT_MIN, TILT_MAX);

            double maxStep = MAX_RATE_DEG_S * dt;
            panCmd  = step(panCmd,  panGoal,  maxStep);
            tiltCmd = step(tiltCmd, tiltGoal, maxStep);

            pan.setPosition(Range.clip(PAN_CENTER + panCmd / SERVO_RANGE_DEG, 0, 1));
            tilt.setPosition(Range.clip(TILT_CENTER + tiltCmd / SERVO_RANGE_DEG, 0, 1));
            record(now, panCmd, tiltCmd);

            telemetry.addData("valid", r != null && r.isValid());
            if (r != null) {
                telemetry.addData("tx / ty", "%.2f / %.2f", r.getTx(), r.getTy());
                telemetry.addData("latency ms", "%.0f", r.getCaptureLatency() + r.getTargetingLatency());
            }
            telemetry.addData("pan / tilt deg", "%.1f / %.1f", panCmd, tiltCmd);
            telemetry.addData("loop ms", "%.1f", dt * 1000);
            telemetry.update();
        }
        dashboard.stopCameraStream();
        ll.stop();
    }
}
