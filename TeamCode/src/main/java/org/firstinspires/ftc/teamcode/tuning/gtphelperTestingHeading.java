
package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.gtp_helper;
import org.firstinspires.ftc.teamcode.gtp_helper_heading;

@Config
@Autonomous(name = "test helper gtp with heading")
public class gtphelperTestingHeading extends OpMode {

    public static double TARGET_X = 600;
    public static double TARGET_Y = 600;

    public static double TARGET_HEADING = 0;

    private gtp_helper_heading drive;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = gtp_helper_heading.fromHardwareMap(hardwareMap, "stanga", "dreapta", "pinpoint", 50, 0);
    }

    @Override
    public void loop() {
        telemetry.addData("targetX", TARGET_X);
        telemetry.addData("targetY", TARGET_Y);

        boolean doneXY = drive.getStatusXY();
        boolean doneHeading = drive.getStatusHeading();
        if(!doneXY) {
            drive.updateXY(TARGET_X, TARGET_Y,telemetry);
        }
        if(!doneHeading && doneXY){
            drive.updateHeading(TARGET_HEADING,telemetry);
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