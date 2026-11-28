package com.qualcomm.robotcore.hardware; import java.util.*;
public class HardwareMap {
 @SuppressWarnings("unchecked") public <T> T get(Class<T> c, String n){ return (T) sim.World.device(n); }
 public <T> List<T> getAll(Class<T> c){ return new ArrayList<>(); }
 public DeviceMapping<DcMotor> dcMotor = new DeviceMapping<>();
 public DeviceMapping<VoltageSensor> voltageSensor = new DeviceMapping<>();
 public static class DeviceMapping<T> implements Iterable<T>{ @SuppressWarnings("unchecked") public T get(String n){return (T) sim.World.device(n);} @SuppressWarnings("unchecked") public Iterator<T> iterator(){ return (Iterator<T>) Collections.singletonList((Object) sim.World.battery).iterator(); } }
}
