
package org.firstinspires.ftc.teamcode.kebab;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


/// actually si cealalta versiune merge, dar am aflat asta mai tarziu
@Config
@Autonomous(name = "kebab test heading fix (versiune care merge)")
public class AutoTestGtpHelperKebab extends OpMode {

    public static double TARGET_X = 600;
    public static double TARGET_Y = 600;

    public static double TARGET_HEADING = 0;

    private gtpHelperKebab drive;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = gtpHelperKebab.fromHardwareMap(hardwareMap, "stanga", "dreapta", "pinpoint", 50, 0);
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
        if (!doneHeading && doneXY) {
            drive.updateHeading(TARGET_HEADING, telemetry);
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