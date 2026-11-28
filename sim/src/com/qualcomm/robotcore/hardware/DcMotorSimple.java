package com.qualcomm.robotcore.hardware; public interface DcMotorSimple { enum Direction{FORWARD,REVERSE} void setDirection(Direction d); void setPower(double p); double getPower(); }
