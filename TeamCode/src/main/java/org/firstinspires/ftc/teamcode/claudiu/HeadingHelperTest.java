package org.firstinspires.ftc.teamcode.claudiu;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.claudiu.TankHeadingHelper;

@TeleOp(name = "Heading Helper Test", group = "Tuning")
@Config
public class HeadingHelperTest extends OpMode {

    // dashboard
    public static double targetHeadingDeg = 0.0;
    public static boolean enabled = true;

    // read at init only -> measure from AXLE MIDPOINT (mm). Placeholders!
    public static double POD_OFFSET_X_MM = 50;
    public static double POD_OFFSET_Y_MM = 0;

    private TankHeadingHelper helper;
    private boolean lastA = false, lastB = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        helper = TankHeadingHelper.fromHardwareMap(
                hardwareMap, "stanga", "dreapta", "pinpoint",
                POD_OFFSET_X_MM, POD_OFFSET_Y_MM);
        telemetry.addLine("A = zero pose | B = reset drift counter");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.a && !lastA) helper.reset();
        if (gamepad1.b && !lastB) helper.resetDrift();
        lastA = gamepad1.a;
        lastB = gamepad1.b;

        if (enabled) {
            helper.update(targetHeadingDeg, telemetry);
        } else {
            helper.stop();
            telemetry.addLine("DISABLED");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        helper.stop();
    }
}