package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DeflectorControls {

    public Servo deflector = null;

    private double _lowestPos = 1.0;
    private double _highestPos = 0.0;
    public double _highGoalPos = 0.24;
    public double _highGoalPosAuto = 0.26;
    public double _powerShotPos = 0.7;
    private double _servoPos;
    private boolean _a = false;
    private boolean _start = false;
    private boolean _back = false;
    private int _currentPosition = 2; // 1 = lowest pos, 2 = middle pos, 3 = highest pos

    boolean _timeCheck = false;
    double _lastTime = 0.0;

    public void initialize(LinearOpMode op) {
        deflector = op.hardwareMap.get(Servo.class, "Deflector");
        _servoPos = _highGoalPos;
        deflector.setPosition(_servoPos);
    }

    public void startControl() {

    }

    public void readController (Gamepad gamepad) {
        if (gamepad.a && !_a) {
            if (_servoPos < 0.45) {
                _servoPos = _powerShotPos;
            } else {
                _servoPos = _highGoalPos;

            }

        } else if (gamepad.start && !_start) {
            if (_servoPos > _highestPos) {
                _servoPos = deflector.getPosition() - 0.05;
            } else {
                _servoPos = _servoPos;
            }
        } else if (gamepad.back && !_back) {
            if (_servoPos < _lowestPos) {
                _servoPos = deflector.getPosition() + 0.05;
            } else {
                _servoPos = _servoPos;
            }
        }
        _a = gamepad.a;
        _start = gamepad.start;
        _back = gamepad.back;
    }

    public void whileOpModeIsActive (LinearOpMode op) {
        this.readController(op.gamepad2);
        deflector.setPosition(_servoPos);
    }

    public void addTelemetry (Telemetry telemetry) {
        if (_servoPos < 0.45) {
            telemetry.addData("Deflecter Pos", "High Goal");
        } else {
            telemetry.addData("Deflecter Pos", "Power Shots");
        }
        telemetry.addData("Deflector Actual Position", deflector.getPosition());
        telemetry.addData("Deflector Target Position", _servoPos);
    }

    public void stop () {
    }
}