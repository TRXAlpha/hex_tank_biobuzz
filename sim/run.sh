#!/bin/bash
# Runs the REAL TeleopAutoAim / Flywheel / FlywheelTuner code on a simulated robot.
# Needs only a JDK (java + javac). Not part of the robot build.
cd "$(dirname "$0")" || exit 1
T=../TeamCode/src/main/java/org/firstinspires/ftc/teamcode
rm -rf out && mkdir out
javac -nowarn -d out -sourcepath src $(find src -name '*.java') \
  $T/Flywheel.java $T/teleop/TeleopAutoAim.java $T/tuning/FlywheelTuner.java && java -cp out sim.Run
