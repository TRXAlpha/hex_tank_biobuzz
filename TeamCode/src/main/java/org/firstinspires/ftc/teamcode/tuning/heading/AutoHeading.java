package org.firstinspires.ftc.teamcode.tuning.heading;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.AutoHelper;

@Autonomous(name="tuning heading")
public class AutoHeading extends OpMode {
    private AutoHelper drive;
    GoBildaPinpointDriver pinpoint;
    private boolean done = false;
    @Override
    public void init() {
        drive = AutoHelper.fromHardwareMap(hardwareMap, "stanga", "dreapta", "pinpoint", 50, 0);
        drive.resetPose();

    }
    @Override
    public void loop() {
        if (!done) {
            drive.updateHeading(359, telemetry);
            done = drive.getStatusHeading();
        } else {
            drive.stop();
        }
        telemetry.addData("heading", drive.getHeading());
        telemetry.addData("done", done);
        telemetry.update();
    }
}
