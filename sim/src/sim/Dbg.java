package sim;
public class Dbg { public static void main(String[] a){
  World.reset(); World.RBAT = 0.05; World.FIRST_TAG = 34; World.OTHER_TAGS = new int[]{30,38,42};
  Run.place(0, -110, 180, 1.03);
  Run.run(org.firstinspires.ftc.teamcode.auto.red.AutoTrasParcareRed::new, 12, (t, op) -> {}, op -> {
    double s = World.t - Run.run0;
    if (s > 5.5 && Math.abs(s*4 - Math.round(s*4)) < 0.011) System.out.printf("t=%.2f %s xy %s along %s headErr %s done %s | L %.2f R %.2f | v %.0f om %.2f%n", s, Run.tl("state"),
       Run.tl("x / y / h"), Run.tl("along"), Run.tl("heading error"), Run.tl("doneXY / doneHeading"), World.m("stanga").power, World.m("dreapta").power, World.v, World.omega); });
}}
