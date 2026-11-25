package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "auto tras tilt jos RED")
public class AutoTrasTiltJosRed extends AutoTrasTiltJos {
    @Override
    protected boolean blue() { return false; }
}
