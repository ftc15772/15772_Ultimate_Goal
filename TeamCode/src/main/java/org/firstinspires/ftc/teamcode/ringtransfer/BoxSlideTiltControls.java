package org.firstinspires.ftc.teamcode.ringtransfer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class BoxSlideTiltControls {

    boolean _timeCheck = false;
    double _lastTime = 0.0;
    private boolean _b = false;
    public boolean _currentBoxInShooterPos = false;
    public boolean _lastBoxInShooterPos = false;

    private double _slideOut = 0.25;
    private double _slideIn = 1.0;
    private double _tiltUp = 0.84;
    private double _tiltDown = 0.0;
    public boolean _sliderIn = true;
    public boolean _tiltingDown = true;

    private BoxSliderControls boxSliderControls = new BoxSliderControls();
    private BoxTilterControls boxTilterControls = new BoxTilterControls();

    public void initialize(LinearOpMode op) {
        boxSliderControls.initialize(op);
        boxTilterControls.initialize(op);
    }

    public void readController (Gamepad gamepad) {
        if (gamepad.b && !_b) {
            _currentBoxInShooterPos =! _currentBoxInShooterPos;
        }
        _b = gamepad.b;
    }

    public void boxIntakePos (double time) {
        boxTilterControls.tilter.setPosition(_tiltDown);
        _tiltingDown = true;
        if ((time - _lastTime) > 0.8) {
            boxSliderControls.slider.setPosition(_slideIn);
            _sliderIn = true;
            _lastBoxInShooterPos = false;
        }

    }

    public void boxShootPos (double time) {
        boxSliderControls.slider.setPosition(_slideOut);
        _sliderIn = false;
        if ((time - _lastTime) > 0.6) {
            boxTilterControls.tilter.setPosition(_tiltUp);
            _tiltingDown = false;
            _lastBoxInShooterPos = true;
        }

    }

    public boolean whileOpModeIsActive (LinearOpMode op, double time) {
        //boxSliderControls.whileOpModeIsActive(op, _tiltingDown);
        //boxTilterControls.whileOpModeIsActive(op, _sliderIn);

        this.readController(op.gamepad2);
        time = time;

        if ((_currentBoxInShooterPos == false) && (_lastBoxInShooterPos == true)) {
            if (_timeCheck == false) {
                _lastTime = time;
                _timeCheck = true;
            }
            boxIntakePos(time);
            if ((_tiltingDown == true) && (_sliderIn == true)) {
                _timeCheck = false;
                _lastBoxInShooterPos = false;
            }
        } else if ((_currentBoxInShooterPos == true) && (_lastBoxInShooterPos == false)) {
            if (_timeCheck == false) {
                _lastTime = time;
                _timeCheck = true;
            }
            boxShootPos(time);
            if ((_tiltingDown == false) && (_sliderIn == false)) {
                _timeCheck = false;
                _lastBoxInShooterPos = true;
            }
        }
        return _currentBoxInShooterPos;
    }

}
