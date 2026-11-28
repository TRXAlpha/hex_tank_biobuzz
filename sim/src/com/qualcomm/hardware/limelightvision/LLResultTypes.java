package com.qualcomm.hardware.limelightvision; import org.firstinspires.ftc.robotcore.external.navigation.*;
public class LLResultTypes { public static class FiducialResult { public int id; public double tx,ty; public Position p;
 public int getFiducialId(){return id;} public Pose3D getTargetPoseCameraSpace(){return new Pose3D(p);} public double getTargetXDegrees(){return tx;} public double getTargetYDegrees(){return ty;} } }
