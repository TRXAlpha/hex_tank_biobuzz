#!/bin/bash
# Runs the REAL calibration + teleop code on a simulated robot. Needs only a JDK (java + javac).
# Not part of the robot build.
cd "$(dirname "$0")" || exit 1
T=../TeamCode/src/main/java/org/firstinspires/ftc/teamcode
rm -rf out && mkdir out
javac -nowarn -d out -sourcepath src $(find src -name '*.java') \
  $T/Calib.java $T/Flywheel.java $T/CellVision.java $T/Power.java \
  $T/teleop/TeleopAutoAim.java $T/tuning/RobotCalibration.java $T/Pozitie.java \
  $T/auto/helpere/AutoHelper.java $T/auto/helpere/AutoHelperTilted.java \
  $T/auto/AutoTrasParcare.java $T/auto/AutoTrasTiltJos.java && java -cp out sim.Run
