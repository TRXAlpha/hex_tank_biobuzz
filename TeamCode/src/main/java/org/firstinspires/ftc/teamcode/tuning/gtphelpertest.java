package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.gtp_helper;
import org.firstinspires.ftc.teamcode.headinghelper;

@Config
@Autonomous(name = "test helper gtp")
public class gtphelpertest extends OpMode {

    public static double TARGET_X = 2370;
    public static double TARGET_Y = 1050;
    public static double TARGET_HEADING = 0;

    private GoBildaPinpointDriver pinpoint;
    private gtp_helper drive;
    private headinghelper turn;

    private boolean driving_done = false;
    private boolean turn_done = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx stanga = hardwareMap.get(DcMotorEx.class, "stanga");
        DcMotorEx dreapta = hardwareMap.get(DcMotorEx.class, "dreapta");

        stanga.setDirection(DcMotorSimple.Direction.REVERSE);
        dreapta.setDirection(DcMotorSimple.Direction.FORWARD);
        stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(50, 0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        drive = new gtp_helper(stanga, dreapta, pinpoint);
        turn = new headinghelper(hardwareMap, stanga, dreapta, pinpoint);
    }

    @Override
    public void loop() {
        if (!driving_done) {
            driving_done = drive.update(TARGET_X, TARGET_Y);
        } else if (!turn_done) {
            turn_done = turn.update(TARGET_HEADING);
        } else {
            drive.stop();
        }

        Pose2D pose = pinpoint.getPosition();

        telemetry.addData("driving done", driving_done);
        telemetry.addData("timed out", drive.isTimedOut());
        telemetry.addData("turning done", turn_done);
        telemetry.addData("targetX", TARGET_X);
        telemetry.addData("targetY", TARGET_Y);
        telemetry.addData("target heading", TARGET_HEADING);
        telemetry.addData("x", "%.1f", pose.getX(DistanceUnit.MM));
        telemetry.addData("y", "%.1f", pose.getY(DistanceUnit.MM));
        telemetry.addData("heading", "%.1f", pose.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }
}