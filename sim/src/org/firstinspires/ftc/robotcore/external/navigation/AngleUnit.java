package org.firstinspires.ftc.robotcore.external.navigation; public enum AngleUnit { RADIANS, DEGREES;
 public static double normalizeDegrees(double d){ while(d>=180)d-=360; while(d<-180)d+=360; return d;} }
