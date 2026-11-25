package com.acmerobotics.dashboard.telemetry; import org.firstinspires.ftc.robotcore.external.Telemetry;
public class MultipleTelemetry implements Telemetry { public MultipleTelemetry(Telemetry... t){}
 public Object addData(String c,Object v){return sim.World.tele.addData(c,v);} public Object addData(String c,String f,Object... a){return sim.World.tele.addData(c,f,a);} public Object addLine(String s){return sim.World.tele.addLine(s);} public boolean update(){return sim.World.tele.update();} }
