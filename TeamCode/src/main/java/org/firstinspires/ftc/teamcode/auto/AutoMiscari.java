package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.auto.helpere.AutoHelper;

// Same setup as AutoTrasParcare, but it only reads the Pinpoint so targets can be measured by hand
@Autonomous(name = "auto miscari", group = "debugging")

public class AutoMiscari extends OpMode {
    private AutoHelper drive;

    private ElapsedTime runtime = new ElapsedTime();

    GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = AutoHelper.fromHardwareMap(hardwareMap, "stanga", "dreapta", "pinpoint", -44.3, -6.6);
        drive.resetPose();
        runtime.reset();
    }

    public void start() {
        drive.resetMove();
        runtime.reset();

    }

    @Override
    public void loop() {
        pinpoint.update();
        drive.refreshPose();

        telemetry.addData("runtime", runtime.seconds());
        telemetry.addData("x", pinpoint.getPosX(DistanceUnit.CM));
        telemetry.addData("y", pinpoint.getPosY(DistanceUnit.CM));
        telemetry.addData("h", pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES));
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
    }
}
