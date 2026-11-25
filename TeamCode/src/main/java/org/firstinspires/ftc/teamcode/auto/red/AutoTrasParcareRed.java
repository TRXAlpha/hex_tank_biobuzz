package org.firstinspires.ftc.teamcode.auto.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoTrasParcare;

@Autonomous(name = "auto tras parcare RED")
public class AutoTrasParcareRed extends AutoTrasParcare {
    @Override
    protected boolean blue() { return false; }
}
