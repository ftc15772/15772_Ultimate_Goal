package org.firstinspires.ftc.teamcode.ringtransfer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BoxFlickerEncoderControlsNew {

    public CRServo flicker = null;
    public DcMotor flickEncoder = null;

    public boolean _flickerKicking = false;
    public boolean _fixOvershoot = false;
    public boolean _fixUndershoot = false;
    private boolean _rightDpad = false;
    private boolean _rightBumper = false;
    private boolean _leftBumper = false;
    private double _currentEncoderValue;
    private double _lastEncoderValue = 0.0;
    private double _timesShots3Rings = 0;
    private double _oneRing = (-8192 / 2) + 1050;
    private boolean _changeTimesShot = false;
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

        if (gamepad.right_bumper && !_rightBumper) {
            _fixUndershoot =! _fixUndershoot;
        }
        _rightBumper = gamepad.right_bumper;


        if (gamepad.left_bumper && !_leftBumper) {
            _fixOvershoot =! _fixOvershoot;
        }
        _leftBumper = gamepad.left_bumper;
    }

    public void whileOpModeIsActive (LinearOpMode op, double time) {
        this.readController(op.gamepad2);
        _currentEncoderValue = flickEncoder.getCurrentPosition();

        if (_flickerKicking == true) {
            /* if (_changeTimesShot == false) {
                _timesShots3Rings += 1;
                _changeTimesShot = true;
            } */
            _timeCheck = false;
            _currentEncoderValue = flickEncoder.getCurrentPosition();


            // if (_currentEncoderValue > (_timesShots3Rings * (3*_oneRing))) {
            if (_currentEncoderValue > (3*_oneRing)) {
                flicker.setPower(-1.0); /*
            } else if ((_currentEncoderValue > (_timesShots3Rings * (4.5*_oneRing))) && (_currentEncoderValue < (_timesShots3Rings * (3*_oneRing)))) {
                flicker.setPower(-0.3); */
            } else {
                _flickerKicking = false;
            }


            /*
            if (_currentEncoderValue > (_lastEncoderValue + (3*_oneRing))) {
                flicker.setPower(-1.0);
            } else if ((_currentEncoderValue > ((_lastEncoderValue + (4.5*_oneRing)))) && (_currentEncoderValue < (_lastEncoderValue + (3*_oneRing)))) {
                flicker.setPower(-0.3);
            } else {
                _flickerKicking = false;
            }
            */

        } else if (_fixOvershoot == true) {
            _timeCheck = false;
            _currentEncoderValue = flickEncoder.getCurrentPosition();
            if ((time - _lastTime) < 0.05) {  //0.05 seconds, NOT milliseconds
                flicker.setPower(0.25);
            } else {
                _fixOvershoot = false;
            }

        } else if (_fixUndershoot == true) {
            _timeCheck = false;
            _currentEncoderValue = flickEncoder.getCurrentPosition();
            if ((time - _lastTime) < 0.05) {  //0.05 seconds, NOT milliseconds
                flicker.setPower(-0.25);
            } else {
                _fixUndershoot = false;
            }

        } else {
            flicker.setPower(0.0);
            flickEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flickEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _changeTimesShot = false;
            _lastEncoderValue = _currentEncoderValue;
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
        telemetry.addData("Flicker Encoder Value", _currentEncoderValue);
    }

    public void stop () {
        flicker.setPower(0.0);
    }
}