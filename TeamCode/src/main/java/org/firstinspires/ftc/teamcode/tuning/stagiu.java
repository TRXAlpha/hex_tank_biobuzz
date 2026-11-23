    package org.firstinspires.ftc.teamcode.tuning;

    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;

    @TeleOp
    public class stagiu extends OpMode {
        DcMotor stagiu;
        @Override
        public void init() {
            stagiu = hardwareMap.dcMotor.get("stagiu");
            stagiu.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        @Override
        public void loop() {
            stagiu.setPower(1);
        }
    }
