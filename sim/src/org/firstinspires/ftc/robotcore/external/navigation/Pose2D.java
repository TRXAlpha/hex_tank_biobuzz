package org.firstinspires.ftc.robotcore.external.navigation; public class Pose2D { double x,y,h; public Pose2D(double x,double y,double h){this.x=x;this.y=y;this.h=h;}
 public double getX(DistanceUnit u){return x;} public double getY(DistanceUnit u){return y;} public double getHeading(AngleUnit u){return h;} }
