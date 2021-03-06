package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ComplicatedSnowplowControls {

    public Servo lftPlow = null;
    public Servo rgtPlow = null;

    private double _lftPosClosed = 0.0;
    private double _rgtPosClosed = 1.0;
    private double _lftPosOpen = 1.0;
    private double _rgtPosOpen = 0.0;
    private boolean _lftOpen = false;
    private boolean _rgtOpen = false;
    private boolean _x = false;
    private boolean _currentOpenPlows = false;
    private boolean _lastOpenPlows = false;

    boolean _timeCheck = false;
    double _lastTime = 0.0;

    public void initialize(LinearOpMode op) {
        lftPlow = op.hardwareMap.get(Servo.class, "LftPlow");
        rgtPlow = op.hardwareMap.get(Servo.class, "RgtPlow");

        lftPlow.setPosition(_lftPosClosed);
        rgtPlow.setPosition(_rgtPosClosed);
    }

    public void startControl(double time) {
        openPlows(time);
        _lftOpen = true;
        _rgtOpen = true;
        _lastOpenPlows = true;
        _currentOpenPlows = true;
    }

    public void readController (Gamepad gamepad) {
        if (gamepad.x && !_x) {
            _currentOpenPlows = !_currentOpenPlows;
        }
        _x = gamepad.x;
    }

    public void openPlows (double time) {
        rgtPlow.setPosition(_rgtPosOpen);
        _rgtOpen = true;
        if ((time - _lastTime) > 0.2) {
            lftPlow.setPosition(_lftPosOpen);
            _lftOpen = true;
            _lastOpenPlows = true;
        }
    }

    public void closePlows (double time) {
        lftPlow.setPosition(_lftPosClosed);
        _lftOpen = false;
        if ((time - _lastTime) > 0.2) {
            rgtPlow.setPosition(_rgtPosClosed);
            _rgtOpen = false;
            _lastOpenPlows = false;
        }

    }

    public void whileOpModeIsActive (LinearOpMode op, double time) {
        this.readController(op.gamepad2);
        time = time;

        if ((_currentOpenPlows == false) && (_lastOpenPlows == true)) {
            if (_timeCheck == false) {
                _lastTime = time;
                _timeCheck = true;
            }
            closePlows(time);
            if ((_lftOpen == false) && (_rgtOpen == false)) {
                _timeCheck = false;
                _lastOpenPlows = false;
            }
        } else if ((_currentOpenPlows == true) && (_lastOpenPlows == false)) {
            if (_timeCheck == false) {
                _lastTime = time;
                _timeCheck = true;
            }
            openPlows(time);
            if ((_lftOpen == true) && (_rgtOpen == true)) {
                _timeCheck = false;
                _lastOpenPlows = true;
            }
        }
    }

    public void addTelemetry (Telemetry telemetry) {
        telemetry.addData("Snowplow Last Time (ms)",_lastTime);
    }

    public void stop () {
    }
}