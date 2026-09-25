
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
    public static boolean shooting = false;
    public static double start_heading = 0;
    public static boolean start_shoot = false;
    public static double timer_shooting = 5;
    public static double timer_start = 2;
    public static double x_parcare = 2370;
    public static double x_shoot = 150;
    public static double x_start = 0;
    public static double y_parcare = 950;
    public static double y_shoot = 950;
    public static double y_start = 0;

    private AutoHelper drive;

    private State state = State.WAIT_START;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime statetimer = new ElapsedTime();

    private enum State { WAIT_START, GO_SHOOT, SHOOT,   GO_PARK, PARKED, GO_START, DONE }



    GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = AutoHelper.fromHardwareMap(hardwareMap, "stanga", "dreapta", "pinpoint", 50, 0);
        drive.resetPose();
        runtime.reset();
    }
public void init_loop(){
    pinpoint.setPosition(new Pose2D(DistanceUnit.CM,0,0, DEGREES, 90));

}
    public void start(){
        state = State.WAIT_START;
        drive.resetMove();
        runtime.reset();
        statetimer.reset();

    }
    @Override
    public void loop() {
        pinpoint.update();
        drive.refreshPose();
        switch (state) {
            case WAIT_START:
                if (statetimer.seconds() >= timer_start) {
                    drive.resetMove();
                    state = State.GO_SHOOT;
                }
                break;

            case GO_SHOOT:
                if (drive.lap(heading_shoot, x_shoot, y_shoot, telemetry)) {
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
                    state=State.PARKED;
                }
                break;

            case PARKED:
                if (statetimer.seconds() >= 5) {
                    drive.resetMove();
                    state = State.GO_START;
                }
                break;

            case GO_START:
                if(drive.lap(start_heading, x_start, y_start, telemetry)){
                    drive.stop();
                    state=State.DONE;
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