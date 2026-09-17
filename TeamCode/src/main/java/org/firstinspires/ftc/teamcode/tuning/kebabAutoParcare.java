package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.AutoHelper;
import org.firstinspires.ftc.teamcode.kebabAutoHelper;

@Config
@Autonomous(name = "gtp+heading kebab")
public class kebabAutoParcare extends OpMode {

    public static double TARGET_X = 2370;
    public static double TARGET_Y = 950;
    public static double TARGET_HEADING = 0;

    private kebabAutoHelper drive;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = kebabAutoHelper.fromHardwareMap(hardwareMap, "stanga", "dreapta", "pinpoint", 50, 0);
        drive.resetPose();
    }

    @Override
    public void loop() {
        telemetry.addData("targetX", TARGET_X);
        telemetry.addData("targetY", TARGET_Y);
        telemetry.addData("targetH", TARGET_HEADING);

        boolean doneXY = drive.getStatusXY();
        boolean doneHeading = drive.getStatusHeading();

        if (!doneXY) {
            drive.updateXY(TARGET_X, TARGET_Y, telemetry);
        }
        if (!doneHeading && doneXY) {
            // ← AICI e modificarea importantă (4 argumente)
            drive.updateHeading(TARGET_HEADING, TARGET_X, TARGET_Y, telemetry);
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