package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.auto.helpere.AutoHelper;

@Config
@Autonomous(name = "auto tras parcare")

public class AutoTrasParcare extends OpMode {
    public static double heading_parcare = -80;
    public static double heading_go_line = 0;

    public static double timer_start = 2;
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
    private AutoHelper drive;
    private DcMotor intake;

    private State state = State.SHOOT;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime statetimer = new ElapsedTime();

    private enum State {SHOOT, GO_LINE, GO_LINE_SLOW, GO_PARK, PARKED, BACK_FROM_LINE, ROTATE_LINE, DONE}


    GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        intake = hardwareMap.get(DcMotor.class, "intake");


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = AutoHelper.fromHardwareMap(hardwareMap, "stanga", "dreapta", "pinpoint", -44.3, -6.6);
        drive.resetPose();
        runtime.reset();
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
                if (drive.lap(heading_go_line, x_go_line, y_go_line, telemetry)) {
                    drive.resetMove();
                    drive.stop();
                    statetimer.reset();
                    intake.setPower(1);
                    state = State.GO_LINE_SLOW;
                }
                break;

            case GO_LINE_SLOW:
                if(drive.lap(heading_go_line_slow, x_go_line_slow, y_go_line_slow, telemetry)){
                    drive.resetMove();
                    drive.stop();
                    statetimer.reset();
                    intake.setPower(0);
                    state = State.BACK_FROM_LINE;
                }
                break;
            case BACK_FROM_LINE:
                if(drive.lap(heading_back_from_line, x_back_from_line, y_back_from_line, telemetry)){
                    drive.resetMove();
                    drive.stop();
                    statetimer.reset();
                    state = State.ROTATE_LINE;
                }
                break;

            case ROTATE_LINE:
                if(drive.lap(heading_rotate_line, x_rotate_line, y_rotate_line, telemetry)){
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
        telemetry.addData("doneXY", drive.getStatusXY());
        telemetry.addData("x", pinpoint.getPosX(DistanceUnit.CM));
        telemetry.addData("y", pinpoint.getPosY(DistanceUnit.CM));
        telemetry.addData("h", pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES));
        telemetry.addData("doneHeading", drive.getStatusHeading());
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
        intake.setPower(0);
    }
}
