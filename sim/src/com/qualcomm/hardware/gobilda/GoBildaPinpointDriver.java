package com.qualcomm.hardware.gobilda; import org.firstinspires.ftc.robotcore.external.navigation.*;
public class GoBildaPinpointDriver { public enum GoBildaOdometryPods{goBILDA_4_BAR_POD} public enum EncoderDirection{FORWARD,REVERSED}
 public enum DeviceStatus{NOT_READY,READY,CALIBRATING,FAULT_X_POD_NOT_DETECTED,FAULT_Y_POD_NOT_DETECTED,FAULT_NO_PODS_DETECTED,FAULT_IMU_RUNAWAY,FAULT_BAD_READ}
 public DeviceStatus getDeviceStatus(){ return sim.World.ODO_FAULT ? DeviceStatus.FAULT_NO_PODS_DETECTED : DeviceStatus.READY; }
 public void setOffsets(double a,double b,DistanceUnit u){} public void setEncoderResolution(GoBildaOdometryPods p){} public void setEncoderDirections(EncoderDirection a,EncoderDirection b){}
 public void resetPosAndIMU(){ sim.World.odoReset(); } public void update(){ sim.World.odoRead(); }
 public Pose2D getPosition(){ return sim.World.odoPose(); } public double getHeadingVelocity(UnnormalizedAngleUnit u){ return u==UnnormalizedAngleUnit.DEGREES?Math.toDegrees(sim.World.omega):sim.World.omega; } public void setPosition(Pose2D p){} }
