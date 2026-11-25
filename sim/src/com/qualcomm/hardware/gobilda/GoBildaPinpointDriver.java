package com.qualcomm.hardware.gobilda; import org.firstinspires.ftc.robotcore.external.navigation.*;
public class GoBildaPinpointDriver { public enum GoBildaOdometryPods{goBILDA_4_BAR_POD} public enum EncoderDirection{FORWARD,REVERSED}
 public void setOffsets(double a,double b,DistanceUnit u){} public void setEncoderResolution(GoBildaOdometryPods p){} public void setEncoderDirections(EncoderDirection a,EncoderDirection b){}
 public void resetPosAndIMU(){ sim.World.odoReset(); } public void update(){ sim.World.odoRead(); }
 public Pose2D getPosition(){ return sim.World.odoPose(); } public double getHeadingVelocity(UnnormalizedAngleUnit u){ return sim.World.omega; } }
