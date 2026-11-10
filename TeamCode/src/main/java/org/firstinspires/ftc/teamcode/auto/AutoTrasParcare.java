package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.auto.helpere.AutoHelper;

@Config
@Autonomous(name = "auto tras parcare")

public class AutoTrasParcare extends OpMode {
    public static double heading_parcare = 0;
    public static double heading_shoot = 0;

    public static double timer_start = 2;
    public static double x_parcare = 2370;
    public static double x_shoot = 150;
    public static double y_parcare = 950;
    public static double y_shoot = 950;
    public static double heading_back_from_line = 0;
    public static double x_back_from_line = 1000;
    public static double y_back_from_line = 0;
    public static double heading_rotate_line = -90;
    public static double x_rotate_line = 840;
    public static double y_rotate_line = 220;
    private AutoHelper drive;

    private State state = State.SHOOT;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime statetimer = new ElapsedTime();

    private enum State {SHOOT, GO_LINE, GO_PARK, PARKED, BACK_FROM_LINE, ROTATE_LINE, DONE}


    GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = AutoHelper.fromHardwareMap(hardwareMap, "stanga", "dreapta", "pinpoint", 95, 45);
        drive.resetPose();
        runtime.reset();
    }

    public void init_loop() {
        pinpoint.setPosition(new Pose2D(DistanceUnit.CM, 0, 0, DEGREES, 90));

    }

    public void start() {
        state = State.SHOOT;
        drive.resetMove();
        runtime.reset();
        statetimer.reset();

    }

    @Override
    public void loop() {
        pinpoint.update();
        drive.refreshPose();
        switch (state) {
            case SHOOT:
                if (statetimer.seconds() >= timer_start) {
                    drive.resetMove();
                    state = State.GO_LINE;
                }
                break;

            case GO_LINE:
                if (drive.lap(heading_shoot, x_shoot, y_shoot, telemetry)) {
                    drive.resetMove();
                    drive.stop();
                    statetimer.reset();
                    state = State.DONE;
                }
                break;
            case BACK_FROM_LINE:
                if(drive.lap(heading_back_from_line, x_back_from_line, y_back_from_line, telemetry)){
                    drive.resetMove();
                    drive.stop();
                    statetimer.reset();
                    state = State.ROTATE_LINE;
                }
            case ROTATE_LINE:
                if(drive.lap(heading_rotate_line, x_rotate_line, y_rotate_line, telemetry)){
                    drive.resetMove();
                    drive.stop();
                    statetimer.reset();
                    state = State.GO_PARK;
                }

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
        telemetry.addData("doneXY", drive.getStatusXY());
        telemetry.addData("doneHeading", drive.getStatusHeading());
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
    }
}
