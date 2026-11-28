package com.qualcomm.robotcore.util; public class Range { public static double clip(double v, double lo, double hi){ return Math.max(lo, Math.min(hi, v)); } }
