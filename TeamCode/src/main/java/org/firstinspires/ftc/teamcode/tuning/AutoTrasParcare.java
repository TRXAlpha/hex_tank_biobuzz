
package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoHelper;

@Config
@Autonomous(name = "auto tras parcare")
public class AutoTrasParcare extends OpMode {
    public static double x_start=0;
    public static double y_start=0;
    public static double start_heading=0;
    public static double timer_start=5;

    public static double x_parcare = 2370;
    public static double y_parcare = 950;
    public static double heading_parcare = 0;
    private static boolean start_parcare=false;

    public static double x_shoot = 0;
    public static double y_shoot = 0;
    public static double heading_shoot = 0;
    public static boolean start_shoot=false;
    public static boolean shooting=false;
    public static double timer_shooting=5;

    private AutoHelper drive;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = AutoHelper.fromHardwareMap(hardwareMap, "stanga", "dreapta", "pinpoint", 50, 0);
        drive.resetPose();
        runtime.reset();
    }

    @Override
    public void loop() {
        double seconds = runtime.seconds();
        if(seconds>=timer_start){start_shoot=true;}
        if(start_shoot){boolean done_gotoshoot = drive.lap(heading_shoot, x_shoot, y_shoot, telemetry);
        if(done_gotoshoot){shooting=true;}}
        //if(shooting&&seconds>=timer_shooting){boolean done_parcare = drive.lap(heading_parcare, x_parcare, y_parcare, telemetry);}
        telemetry.addData("timer", seconds);
        telemetry.addData("shooting", shooting);
        telemetry.addData("targetX", x_parcare);
        telemetry.addData("targetY", y_parcare);
        telemetry.addData("doneXY", drive.getStatusXY());
        telemetry.addData("doneHeading", drive.getStatusHeading());
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
    }
}