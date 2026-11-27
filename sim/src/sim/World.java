package sim;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import java.util.*;

/** Physics world: tank drive, pot turret, flywheel + battery sag, Limelight geometry. Units mm, s, rad. */
public class World {
    public static double t = 0;
    public static Gamepad pad = new Gamepad();
    public static Tele tele = new Tele();
    public static Random rnd = new Random(1);

    // ---- truth ----
    public static double x, y, h, v, omega;                 // robot pose (field), forward speed, yaw rate
    public static double potV = 1.03, potW = 0;             // turret pot volts + V/s
    public static double fly = 0;                           // flywheel tps
    public static double vbat = 12.6, iTot = 0;
    // ---- params ----
    public static double VOC = 12.6, RBAT = 0.04;           // battery open-circuit volts, ohm (battery+wiring)
    public static double CAM_PITCH_TRUE = 25, CAM_FWD = 30, CAM_H = 300;
    public static double TAG_H = 700;                       // tag centers above floor (mm)
    public static double RANGE_SCALE = 1.0;                 // 2.0 = Limelight tag size set to 165.1 instead of 82.55
    public static double ODO_SCALE = 1.0, ODO_HEAD_DRIFT = 0;   // odometry errors (scale, rad/s)
    public static double HX = 0, HY = 0, ROW = 0;           // CELL center (field) + row direction (rad)
    public static int FIRST_TAG = 38;                       // blue audience by default
    public static int[] OTHER_TAGS = {30, 34, 42};          // other clusters (placed elsewhere)
    public static boolean CAMERA_BLOCKED = false, DEBUG = false;
    public static double FOV_X = 27.25, FOV_Y = 21, MAX_RANGE = 3500, FPS = 40;
    public static double DRIVE_MAX = 1500, TRACK = 380;     // mm/s at 12 V, track width
    public static double DRIVE_FRIC = 0.05, TURN_FRIC = 0.25;  // power lost to friction (skid-steer turning is hard)
    public static double TURRET_GAIN = 2.5;                 // V/s at power 1 (pot space)
    public static double FLY_VPV = 183, FLY_FRIC_V = 0.4, FLY_TAU = 0.6;  // tps per volt, friction volts, time const

    // ---- odometry frame ----
    static double ox, oy, oh, odx, ody, odh;               // origin at reset, odometry reading
    public static void odoReset(){ ox = x; oy = y; oh = h; odx = ody = odh = 0; }
    public static void odoRead(){}
    public static Pose2D odoPose(){ return new Pose2D(odx, ody, wrap(odh)); }

    // ---- shots ----
    public static int shots = 0; public static List<Double> shotPct = new ArrayList<>(); public static double feedT = 0;
    public static double lastTarget = 0, shotAimMax = 0;

    // ---- devices ----
    public static class Motor implements DcMotorEx {
        public String name; public double power, amps; public Direction dir = Direction.FORWARD; public ZeroPowerBehavior zpb = ZeroPowerBehavior.FLOAT;
        Motor(String n){name=n;}
        public void setDirection(Direction d){dir=d;} public void setPower(double p){power=Math.max(-1,Math.min(1,p));} public double getPower(){return power;}
        public void setMode(RunMode m){} public void setZeroPowerBehavior(ZeroPowerBehavior z){zpb=z;}
        public double getVelocity(){ return fly + rnd.nextGaussian()*5; }
        public double getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit u){ return amps + rnd.nextGaussian()*0.1; }
    }
    public static Map<String,Object> dev = new HashMap<>();
    public static VoltageSensor battery = () -> vbat + rnd.nextGaussian()*0.02;
    public static LLResult llLatest = null;
    static double nextFrame = 0;
    static { for (String n : new String[]{"stanga","dreapta","intake","stagiu","turret","turelaD","turelaS"}) dev.put(n, new Motor(n));
             dev.put("potTurela", new AnalogInput()); dev.put("limelight", new Limelight3A());
             dev.put("pinpoint", new com.qualcomm.hardware.gobilda.GoBildaPinpointDriver()); }
    public static Object device(String n){ Object o = dev.get(n); if (o == null) throw new RuntimeException("no device " + n); return o; }
    static Motor m(String n){ return (Motor) dev.get(n); }

    public static double potVolts(){ return potV + rnd.nextGaussian()*0.002; }

    // ---- turret angle model = same pot formula as the code (assumes calibration is right) ----
    public static double V_FRONT = 0.663, V_CENTER = 1.030, V_BACK = 1.500; public static boolean LATERAL_IS_LEFT = false;
    public static double turretDeg(double volts){
        double p = potAngleDeg(volts), pC = potAngleDeg(V_CENTER);
        double old = p <= pC ? (p - pC) / (pC - potAngleDeg(V_BACK)) * 90.0 : (p - pC) / (potAngleDeg(V_FRONT) - pC) * 90.0;
        double d = 90.0 - old; return LATERAL_IS_LEFT ? d : -d; }
    static double potAngleDeg(double volts){ if (volts < 1e-3) return 270.0; double b = 270.0*volts+445.5, c = 120285.0-36450.0*volts; return (b-Math.sqrt(b*b-4.0*volts*c))/(2.0*volts); }

    public static double camHeading(){ return h + Math.toRadians(turretDeg(potV)); }
    public static double[] tagPos(int i){ double[] off = {-165.1,-69.85,69.85,165.1}; return new double[]{HX + off[i]*Math.cos(ROW), HY + off[i]*Math.sin(ROW)}; }

    /** True aim error (deg) = turret direction vs direction pivot -> CELL center. */
    public static double aimErr(){ double want = Math.atan2(HY - y, HX - x); return Math.toDegrees(wrap(want - camHeading())); }
    public static double trueDistCm(){ return Math.hypot(HX - x, HY - y) / 10.0; }

    // ---- physics step ----
    public static void step(double dt){
        // battery
        // physical motor power = logical power, flipped by REVERSE; the two sides are mirror-mounted:
        // forward needs left motor -1 and right motor +1 (auto: stanga REVERSE; teleop: dreapta REVERSE)
        double pl = m("stanga").power * (m("stanga").dir == DcMotorSimple.Direction.REVERSE ? -1 : 1);
        double pr = m("dreapta").power * (m("dreapta").dir == DcMotorSimple.Direction.REVERSE ? -1 : 1);
        double vL = -pl, vR = pr;
        double cmdL = vL, cmdR = vR;                                   // applied (for the motor current)
        // skid-steer friction: turning in place needs TURN_FRIC power before the robot turns, driving DRIVE_FRIC
        double cm = (vL + vR) / 2, df = (vR - vL) / 2;
        cm = Math.signum(cm) * Math.max(0, Math.abs(cm) - DRIVE_FRIC);
        df = Math.signum(df) * Math.max(0, Math.abs(df) - TURN_FRIC);
        vL = cm - df; vR = cm + df;
        double sp = DRIVE_MAX / 12.0;                                   // mm/s per volt
        double tl = vL*vbat*sp, tr = vR*vbat*sp;                       // target side speeds
        double l = v - omega*TRACK/2, r = v + omega*TRACK/2;
        l += (tl - l) * dt / 0.12; r += (tr - r) * dt / 0.12;
        v = (l + r) / 2; omega = (r - l) / TRACK;
        double iL = cmdL == 0 ? 0 : 2*Math.abs(cmdL*vbat - l/sp)/1.3, iR = cmdR == 0 ? 0 : 2*Math.abs(cmdR*vbat - r/sp)/1.3;   // 0 power = brake, no battery current
        m("stanga").amps = iL; m("dreapta").amps = iR;
        double iDrive = iL + iR;
        x += v*Math.cos(h)*dt; y += v*Math.sin(h)*dt; h = wrap(h + omega*dt);
        // odometry (in reset frame, with errors)
        double dxr = v*Math.cos(h)*dt*ODO_SCALE, dyr = v*Math.sin(h)*dt*ODO_SCALE;
        double c = Math.cos(-oh), s = Math.sin(-oh);
        odx += dxr*c - dyr*s; ody += dxr*s + dyr*c; odh = wrap(h - oh + ODO_HEAD_DRIFT*t);
        // turret
        double p = m("turret").power; double pe = Math.abs(p) < 0.08 ? 0 : p - Math.copySign(0.08, p);
        potW += (pe*TURRET_GAIN*vbat/12 - potW) * dt / 0.05;
        potV += potW*dt; if (potV < 0.45) { potV = 0.45; potW = 0; } if (potV > 1.8) { potV = 1.8; potW = 0; }
        double iTur = Math.abs(p)*4;
        // flywheel (turelaD/S share one wheel)
        double pf = m("turelaD").power;
        double emf = fly / FLY_VPV;
        double drive = pf*vbat;
        if (pf == 0 && m("turelaD").zpb == DcMotor.ZeroPowerBehavior.FLOAT) fly -= Math.signum(fly) * FLY_FRIC_V * FLY_VPV / FLY_TAU * dt;   // coast
        else fly += ((drive - FLY_FRIC_V*Math.signum(fly) ) * FLY_VPV - fly) * dt / FLY_TAU;                                                   // drive / brake
        if (fly < 0 && pf <= 0) fly = Math.max(fly, 0);
        double iFly = 2*Math.max(0, drive - emf)/1.3;
        m("turelaD").amps = iFly / 2; m("turelaS").amps = iFly / 2;
        m("intake").amps = Math.abs(m("intake").power) * 1.5; m("stagiu").amps = Math.abs(m("stagiu").power) * 1.5;
        // feeder -> shots: a ball leaves after 80 ms of feeding, next ball 250 ms later
        double iFeed = (Math.abs(m("intake").power) + Math.abs(m("stagiu").power))*1.5;
        if (m("stagiu").power > 0.5) { feedT += dt; if (feedT > 0.08) { if (DEBUG) System.out.printf("      shot t=%.2f aimErr=%.2f turretVel=%.0f deg/s omega=%.0f deg/s%n", t, aimErr(), potW*(turretDeg(potV+0.001)-turretDeg(potV))/0.001, Math.toDegrees(omega)); shots++; shotAimMax = Math.max(shotAimMax, Math.abs(aimErr())); shotPct.add(lastTarget > 0 ? fly/lastTarget*100 : 0); fly *= 0.94; feedT = -0.25; } }
        else feedT = Math.min(feedT, 0) ;
        if (m("stagiu").power <= 0.5 && feedT < 0) feedT = Math.min(0, feedT + dt);
        iTot = iDrive + iTur + iFly + iFeed + 1.0;
        vbat = VOC - RBAT*iTot;
        // limelight frames
        if (t >= nextFrame) { nextFrame = t + 1.0/FPS; frame(); }
        t += dt;
    }

    static List<double[]> pend = new ArrayList<>();   // frames waiting to be delivered (tCap, result)
    static List<LLResult> pendR = new ArrayList<>();
    static void frame(){
        LLResult r = new LLResult(); r.tFrame = t;
        if (!CAMERA_BLOCKED) {
            double ch = camHeading(), cxp = x + CAM_FWD*Math.cos(ch), cyp = y + CAM_FWD*Math.sin(ch);
            double pr = Math.toRadians(CAM_PITCH_TRUE);
            List<int[]> all = new ArrayList<>();
            for (int i = 0; i < 4; i++) all.add(new int[]{FIRST_TAG + i, i, 0});
            for (int base : OTHER_TAGS) for (int i = 0; i < 4; i++) all.add(new int[]{base + i, i, 1});
            for (int[] tg : all) {
                double tx0, ty0;
                if (tg[2] == 0) { double[] q = tagPos(tg[1]); tx0 = q[0]; ty0 = q[1]; }
                else { tx0 = HX + 3000 + tg[0]*10; ty0 = HY + 2000; }      // other clusters far away
                double dx = tx0 - cxp, dy = ty0 - cyp, dz = TAG_H - CAM_H;
                double f = dx*Math.cos(ch) + dy*Math.sin(ch), lft = -dx*Math.sin(ch) + dy*Math.cos(ch);
                double zc = f*Math.cos(pr) + dz*Math.sin(pr), up = -f*Math.sin(pr) + dz*Math.cos(pr), xc = -lft;
                if (zc <= 0) continue;
                double tx = Math.toDegrees(Math.atan2(xc, zc)), ty = Math.toDegrees(Math.atan2(up, zc));
                double range = Math.sqrt(dx*dx + dy*dy + dz*dz);
                if (Math.abs(tx) > FOV_X || Math.abs(ty) > FOV_Y || range > MAX_RANGE) continue;
                double rs = RANGE_SCALE * (1 + rnd.nextGaussian()*0.01);
                LLResultTypes.FiducialResult fr = new LLResultTypes.FiducialResult();
                fr.id = tg[0]; fr.tx = tx + rnd.nextGaussian()*0.1; fr.ty = ty + rnd.nextGaussian()*0.1;
                fr.p = new Position(xc*rs, -up*rs, zc*rs);
                r.fid.add(fr);
            }
        }
        pend.add(new double[]{t}); pendR.add(r);
        // deliver after 20 ms (capture + targeting latency)
    }
    public static void deliver(){
        while (!pend.isEmpty() && t - pend.get(0)[0] >= 0.020) { llLatest = pendR.get(0); pend.remove(0); pendR.remove(0); }
    }

    public static double wrap(double a){ return Math.atan2(Math.sin(a), Math.cos(a)); }

    public static void reset(){
        t = 0; x = y = h = v = omega = 0; potV = 1.03; potW = 0; fly = 0; vbat = VOC; iTot = 0; nextFrame = 0; llLatest = null;
        pend.clear(); pendR.clear(); shots = 0; shotAimMax = 0; shotPct.clear(); feedT = 0; pad = new Gamepad(); tele = new Tele(); rnd = new Random(1);
        for (Object o : dev.values()) if (o instanceof Motor) { ((Motor) o).power = 0; ((Motor) o).dir = DcMotorSimple.Direction.FORWARD; ((Motor) o).amps = 0; ((Motor) o).zpb = DcMotor.ZeroPowerBehavior.FLOAT; }
        CAMERA_BLOCKED = false;
    }

    public static class Tele implements Telemetry {
        public Map<String,String> last = new LinkedHashMap<>(), cur = new LinkedHashMap<>(); public List<String> lines = new ArrayList<>(), curLines = new ArrayList<>();
        public Object addData(String c,Object v){cur.put(c,String.valueOf(v));return null;}
        public Object addData(String c,String f,Object... a){cur.put(c,String.format(f,a));return null;}
        public Object addLine(String s){curLines.add(s);return null;}
        public boolean update(){last=cur;lines=curLines;cur=new LinkedHashMap<>();curLines=new ArrayList<>();return true;}
    }
}
