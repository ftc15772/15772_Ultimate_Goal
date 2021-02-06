package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DeflectorControls {

    public Servo deflector = null;

    private double _lowestPos = 1.0;
    private double _highestPos = 0.0;
    private double _highGoalPos= 0.3;
    private double _servoPos = _highGoalPos;
    private boolean _a = false;
    private boolean _start = false;
    private boolean _back = false;
    private int _currentPosition = 2; // 1 = lowest pos, 2 = middle pos, 3 = highest pos

    boolean _timeCheck = false;
    double _lastTime = 0.0;

    public void initialize(LinearOpMode op) {
        deflector = op.hardwareMap.get(Servo.class, "Deflector");
        deflector.setPosition(_servoPos);
    }

    public void startControl() {

    }

    public void readController (Gamepad gamepad) {
        if (gamepad.a && !_a) {
            _servoPos = _highGoalPos;
        } else if (gamepad.start && !_start) {
            _servoPos -= 0.05;
        } else if (gamepad.back && !_back) {
            _servoPos += 0.05;
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
        telemetry.addData("Deflector Position", deflector.getPosition());
    }

    public void stop () {
    }
}