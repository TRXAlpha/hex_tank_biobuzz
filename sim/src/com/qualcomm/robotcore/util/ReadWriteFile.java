package com.qualcomm.robotcore.util; public class ReadWriteFile { public static java.util.Map<String,String> FS = new java.util.HashMap<>();
 public static void writeFile(java.io.File f,String s){FS.put(f.getName(),s);} public static String readFile(java.io.File f){ String s=FS.get(f.getName()); if(s==null) throw new RuntimeException("no file"); return s;} }
