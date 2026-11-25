package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "auto tras parcare RED")
public class AutoTrasParcareRed extends AutoTrasParcare {
    @Override
    protected boolean blue() { return false; }
}
