package org.firstinspires.ftc.teamcode.ringtransfer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BoxFlickerEncoderControls {

    public CRServo flicker = null;
    public DcMotor flickEncoder = null;

    public boolean _flickerKicking = false;
    private boolean _rightDpad = false;
    private boolean _rightBumper;
    private double _encoderValue;
    private double _lastEncoderValue = 0.0;
    private double _oneRing = -4150.0;
    boolean _timeCheck = false;
    double _lastTime = 0.0;

    public void initialize(LinearOpMode op) {
        flicker = op.hardwareMap.get(CRServo.class, "ShooterFlick");
        flickEncoder = op.hardwareMap.get(DcMotor.class, "Intake");
        flickEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flickEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flicker.setPower(0.0);
    }

    public void startControl() {
    }

    public void readController (Gamepad gamepad) {
        if (gamepad.dpad_right && !_rightDpad) {
            _flickerKicking =! _flickerKicking;
        }
        _rightDpad = gamepad.dpad_right;

        _rightBumper = gamepad.right_bumper;
    }

    public void whileOpModeIsActive (LinearOpMode op, double time) {
        this.readController(op.gamepad2);
        _encoderValue = flickEncoder.getCurrentPosition();

        if (_flickerKicking == true) {
            _timeCheck = false;
            _encoderValue = flickEncoder.getCurrentPosition();
            if ((time - _lastTime) < 1.0) {
                flicker.setPower(-1.0);
            } else {
                _flickerKicking = false;
            }

            /* if (_encoderValue > ((_lastEncoderValue + (3*_oneRing)) - 500)) {
                flicker.setPower(-1.0);
            } else if ((_encoderValue > ((_lastEncoderValue + (3*_oneRing)) - 300)) && (_encoderValue < ((_lastEncoderValue + (3*_oneRing)) - 500))) {
                flicker.setPower(-0.1);
            } else if (_encoderValue < ((_lastEncoderValue + (3*_oneRing)) - 300)) {
                flicker.setPower(0.1);
            } else {
                _flickerKicking = false;
            }
             */

        } else if (_rightBumper == true) {
            flicker.setPower(-0.25);
        } else {
            flicker.setPower(0.0);
            _lastEncoderValue = _encoderValue;
            _flickerKicking = false;
            _timeCheck = true;
        }

        if (_timeCheck == true) {
            _lastTime = time;
        } else {
            _lastTime = _lastTime;
        }
    }

    public void addTelemetry (Telemetry telemetry) {
        telemetry.addData("Flicker Kicking", _flickerKicking);
        telemetry.addData("Flicker Encoder Value", _encoderValue);
    }

    public void stop () {
        flicker.setPower(0.0);
    }
}