package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.gtpHelperKebab;
import org.firstinspires.ftc.teamcode.gtpHelperKebabCuReset;

/// test pentru patrat folosind gtpHelperKebab
@Config
@Autonomous(name = "test patrat kebab helper")
public class AutoTestPatratGtpHelperKebab extends OpMode {

    public static double TARGET_X_1 = 600;
    public static double TARGET_Y_1 = 0;

    public static double TARGET_X_2 = 600;
    public static double TARGET_Y_2 = 600;

    public static double TARGET_X_3 = 0;
    public static double TARGET_Y_3 = 600;

    public static double TARGET_X_4 = 0;
    public static double TARGET_Y_4 = 0;

    public static double TARGET_HEADING = 0;

    private int counter = 0;

    private gtpHelperKebabCuReset drive;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = gtpHelperKebabCuReset.fromHardwareMap(hardwareMap, "stanga", "dreapta", "pinpoint", 50, 0);
    }

    @Override
    public void loop() {
        double TARGET_X = -1;
        double TARGET_Y = -1;

        switch (counter){
            case 0:
                TARGET_X=TARGET_X_1;
                TARGET_Y=TARGET_Y_1;
                break;
            case 1:
                TARGET_X=TARGET_X_2;
                TARGET_Y=TARGET_Y_2;
                break;
            case 2:
                TARGET_X=TARGET_X_3;
                TARGET_Y=TARGET_Y_3;
                break;
            case 3:
                TARGET_X=TARGET_X_4;
                TARGET_Y=TARGET_Y_4;
                break;
        }

        if(TARGET_X == -1 || TARGET_Y == -1){
            telemetry.addData("Target x and target y are not set.","");
            telemetry.update();
            return;
        }

        telemetry.addData("targetX", TARGET_X);
        telemetry.addData("targetY", TARGET_Y);
        telemetry.addData("counter",counter);

        boolean doneXY = drive.getStatusXY();
        boolean doneHeading = drive.getStatusHeading();

        if(counter == 3) {
            if (doneHeading && doneXY) {
                counter++;
                drive.reset();
            }
        }else{
            if(doneXY){
                counter++;
                drive.reset();
            }
        }

        if(!doneXY) {
            drive.updateXY(TARGET_X, TARGET_Y,telemetry);
        }
        if(counter ==3) {
            if (!doneHeading && doneXY) {
                drive.updateHeading(TARGET_HEADING, telemetry);
            }
        }
        telemetry.addData("doneXY", doneXY);
        telemetry.addData("doneHeading", doneHeading);
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
    }
}
