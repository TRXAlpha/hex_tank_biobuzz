package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="pinpoint debug")
public class pinpointdebug extends OpMode {
    public GoBildaPinpointDriver pinpoint;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(55, 0, DistanceUnit.MM); // change these to your real offsets
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.resetPosAndIMU();

    }

    @Override
    public void loop() {
        pinpoint.update();
        telemetry.addData("Current", "%.1f°", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Current", "%.1f mm", pinpoint.getPosition().getX(DistanceUnit.MM));
        telemetry.addData("Current", "%.1f mm", pinpoint.getPosition().getY(DistanceUnit.MM));
        telemetry.update();
    }
}
