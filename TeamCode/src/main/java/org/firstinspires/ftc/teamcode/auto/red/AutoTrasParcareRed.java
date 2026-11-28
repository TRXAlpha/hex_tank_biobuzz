package org.firstinspires.ftc.teamcode.auto.red;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Pozitie;
import org.firstinspires.ftc.teamcode.auto.helpere.AutoHelper;

// Aims at tag cluster 34 (RED audience CELL).
@Config
@Autonomous(name = "auto tras parcare RED", group = "RED")
public class AutoTrasParcareRed extends OpMode {
    public static double SHOOT_MS = 5000;   // gated feed window after flywheel at speed + turret on target

    public static double heading_parcare = -80;
    public static double heading_go_line = 0;
    public static double x_parcare = 1190;
    public static double x_go_line = 1000;
    public static double y_parcare = -1980;
    public static double y_go_line = 10;
    public static double heading_back_from_line = 0;
    public static double x_back_from_line = 670;
    public static double y_back_from_line = 10;
    public static double heading_rotate_line = -75;
    public static double x_rotate_line = 730;
    public static double y_rotate_line = -10;
    public static double heading_go_line_slow = 0;
    public static double y_go_line_slow = 0;
    public static double x_go_line_slow = 1200;
    public static double STAGIU_POWER = 1;

    private AutoHelper drive;
    private DcMotor intake, stagiu;

    private State state = State.SHOOT;
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime statetimer = new ElapsedTime();

    private enum State {SHOOT, JIGGLE_BACK, JIGGLE_FORWARD, GO_LINE, GO_LINE_SLOW, GO_PARK, PARKED, BACK_FROM_LINE, ROTATE_LINE, DONE}

    @Override
    public void init() {
    stagiu = hardwareMap.dcMotor.get("stagiu");
        stagiu.setDirection(DcMotorSimple.Direction.REVERSE);
        stagiu.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); intake = hardwareMap.get(DcMotor.class, "intake");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = AutoHelper.fromHardwareMap(hardwareMap, "stanga", "dreapta", "pinpoint", -44.3, -6.6, 34);
        drive.resetPose();
        Pozitie.clear();                   // no old pose leaks into teleop
        runtime.reset();
    }

    @Override
    public void init_loop() {
        drive.turretInitLoop(telemetry);   // turret -> center
        telemetry.update();
    }

    @Override
    public void start() {
        state = State.SHOOT;
        drive.resetMove();
        drive.startTurret();               // seed hive along current turret direction
        runtime.reset();
        statetimer.reset();
    }

    @Override
    public void loop() {
        drive.update(telemetry);           // pinpoint once + turret tracking, every loop
        Pozitie.setMmDeg(drive.getX(), drive.getY(), drive.getHeading());   // hand pose to teleop
        switch (state) {
            case SHOOT:
                if (drive.shootFor(SHOOT_MS, telemetry)) {
                    drive.resetMove();
                    statetimer.reset();
                    state = State.GO_LINE;
                }
                break;

            case GO_LINE:
                intake.setPower(1);
                if (drive.lap(heading_go_line, x_go_line, y_go_line, telemetry)) {
                    drive.resetMove();
                    drive.stop();
                    statetimer.reset();
                    state = State.GO_LINE_SLOW;
                }
                break;

            case GO_LINE_SLOW:
                if (drive.lap(heading_go_line_slow, x_go_line_slow, y_go_line_slow, telemetry)) {
                    drive.resetMove();
                    drive.stop();
                    statetimer.reset();
                    state = State.BACK_FROM_LINE;
                }
                break;

            case BACK_FROM_LINE:
                if (drive.lap(heading_back_from_line, x_back_from_line, y_back_from_line, telemetry)) {
                    drive.resetMove();
                    drive.stop();
                    statetimer.reset();

                    state = State.JIGGLE_FORWARD;
                }
                break;

            case JIGGLE_FORWARD:
                if(drive.lap(heading_go_line_slow, x_go_line_slow, y_go_line_slow, telemetry)){
                    drive.resetMove();
                    drive.stop();
                    statetimer.reset();
                    state = State.JIGGLE_BACK;
                }
                break;

            case JIGGLE_BACK:
                if(drive.lap(heading_back_from_line, x_back_from_line, y_back_from_line, telemetry)){
                    drive.resetMove();
                    drive.stop();
                    statetimer.reset();
                    state = State.ROTATE_LINE;
                }
                break;

            case ROTATE_LINE:
                intake.setPower(0);
                if (drive.lap(heading_rotate_line, x_rotate_line, y_rotate_line, telemetry)) {
                    drive.resetMove();
                    drive.stop();
                    statetimer.reset();
                    state = State.GO_PARK;
                }
                break;

            case GO_PARK:
                if (drive.lap(heading_parcare, x_parcare, y_parcare, telemetry)) {
                    drive.resetMove();
                    drive.stop();
                    statetimer.reset();
                    state = State.PARKED;
                }
                break;

            case PARKED:
                if (statetimer.seconds() >= 5) {
                    drive.resetMove();
                    state = State.DONE;
                }
                break;

            case DONE:
                drive.stop();
                break;
        }

        telemetry.addData("state", state);
        telemetry.addData("runtime", runtime.seconds());
        telemetry.addData("stateTime", statetimer.seconds());
        telemetry.addData("doneXY / doneHeading", "%b / %b", drive.getStatusXY(), drive.getStatusHeading());
        telemetry.addData("x / y / h", "%.0f / %.0f / %.1f", drive.getX(), drive.getY(), drive.getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stopAll();
        intake.setPower(0);
        Pozitie.saveFile();                // backup if the robot app restarts
    }
}