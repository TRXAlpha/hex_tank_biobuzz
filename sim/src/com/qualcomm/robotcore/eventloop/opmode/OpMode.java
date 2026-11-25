package com.qualcomm.robotcore.eventloop.opmode; import com.qualcomm.robotcore.hardware.*; import org.firstinspires.ftc.robotcore.external.Telemetry;
public abstract class OpMode { public Gamepad gamepad1 = sim.World.pad; public HardwareMap hardwareMap = new HardwareMap(); public Telemetry telemetry = sim.World.tele;
 public abstract void init(); public void init_loop(){} public void start(){} public abstract void loop(); public void stop(){} }
