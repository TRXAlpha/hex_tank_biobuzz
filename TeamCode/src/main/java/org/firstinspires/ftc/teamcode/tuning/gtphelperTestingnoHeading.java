
package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.gtp_helper;

@Config
@Autonomous(name = "test helper gtp without heading")
public class gtphelperTestingnoHeading extends OpMode {

    public static double TARGET_X = 600;
    public static double TARGET_Y = 600;

    private gtp_helper drive;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = gtp_helper.fromHardwareMap(hardwareMap, "stanga", "dreapta", "pinpoint", 50, 0);
    }

    @Override
    public void loop() {
        telemetry.addData("targetX", TARGET_X);
        telemetry.addData("targetY", TARGET_Y);

        boolean done = drive.update(TARGET_X, TARGET_Y,telemetry);
        telemetry.addData("done", done);
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
    }
}