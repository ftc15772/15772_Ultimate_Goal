package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Flicker Test", group = "Linear Opmode")
@Disabled
public class FlickerTest extends LinearOpMode {

    public CRServo flicker = null;
    public DcMotor flickEncoder = null;

    private boolean _flickerKicking = false;
    private boolean _startButton = false;
    private double _encoderValue;

    @Override
    public void runOpMode() {
        flicker = hardwareMap.get(CRServo.class, "ShooterFlick");
        flickEncoder = hardwareMap.get(DcMotor.class, "Intake");
        flickEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flickEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flicker.setPower(0.0);

        waitForStart();

        while(opModeIsActive()) {
            _encoderValue = flickEncoder.getCurrentPosition();

            telemetry.addData("Flicker Encoder Value", _encoderValue);

            telemetry.update();
            idle();
        }
        flicker.setPower(0.0);
    }
}
