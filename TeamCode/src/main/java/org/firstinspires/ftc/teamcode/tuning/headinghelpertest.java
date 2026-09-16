package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.headinghelper;

@Autonomous(name = "heading helper test")
public class headinghelpertest extends OpMode {

    double TARGET_HEADING = 90;
    private headinghelper turn;
    @Override
    public void init() {
        turn = headinghelper.fromHardwareMap(hardwareMap, "stanga", "dreapta", "pinpoint", 50, 0);
    }

    @Override
    public void loop() {
        boolean done = turn.update(TARGET_HEADING);
        telemetry.addData("error", turn.getError());
        telemetry.addData("power", turn.getTurnPower());
        telemetry.update();
    }
}
