package com.qualcomm.hardware.limelightvision; public class LLResult { public long stale; public double tFrame; public java.util.List<LLResultTypes.FiducialResult> fid = new java.util.ArrayList<>();
 public long getStaleness(){return (long)((sim.World.t - tFrame)*1000) - 20;} public boolean isValid(){return !fid.isEmpty();} public double getCaptureLatency(){return 12;} public double getTargetingLatency(){return 8;}
 public java.util.List<LLResultTypes.FiducialResult> getFiducialResults(){return fid;} }
