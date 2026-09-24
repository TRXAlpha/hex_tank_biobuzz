package org.firstinspires.ftc.teamcode.tuning.heading;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoHelper;

@Autonomous(name = "tuning heading 4ori")
public class AutoHeading4ori extends OpMode {
    private static final int TURNS = 4;
    private static final double PAUSE_S = 1.0;

    private AutoHelper drive;
    private final ElapsedTime pauseTimer = new ElapsedTime();
    private int step = 0;
    private boolean waiting = false;


    @Override
    public void init() {
        drive = AutoHelper.fromHardwareMap(hardwareMap, "stanga", "dreapta", "pinpoint", 50, 0);
    }

    @Override
    public void start() {
        step = 0;
        waiting = false;
        drive.resetMove();
    }

    @Override
    public void loop() {
        double target = 90 * (step + 1);

        if (step >= TURNS) {
            drive.stop();
        } else if (waiting) {
            drive.stop();
            if (pauseTimer.seconds() >= PAUSE_S) {
                waiting = false;
                drive.resetMove();
            }
        } else {
            drive.updateHeading(target, telemetry);
            if (drive.getStatusHeading()) {
                step++;
                waiting = true;
                pauseTimer.reset();
            }
        }

        telemetry.addData("step", step);
        telemetry.addData("target", target);
        telemetry.addData("waiting", waiting);
        telemetry.addData("heading", drive.getHeading());
        telemetry.addData("x", drive.getX());
        telemetry.addData("y", drive.getY());
        telemetry.update();
    }
}