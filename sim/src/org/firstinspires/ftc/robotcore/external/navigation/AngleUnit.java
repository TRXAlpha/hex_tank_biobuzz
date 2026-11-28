package org.firstinspires.ftc.robotcore.external.navigation; public enum AngleUnit { RADIANS, DEGREES;
 public static double normalizeDegrees(double d){ while(d>=180)d-=360; while(d<-180)d+=360; return d;}
 public static double normalizeRadians(double r){ return Math.atan2(Math.sin(r), Math.cos(r)); } }
