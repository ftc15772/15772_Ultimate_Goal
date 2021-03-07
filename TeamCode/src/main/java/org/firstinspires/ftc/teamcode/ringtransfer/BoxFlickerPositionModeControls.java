package org.firstinspires.ftc.teamcode.ringtransfer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BoxFlickerPositionModeControls {

    public Servo flicker = null;


    private boolean _rightDpad = false;
    private boolean _rightBumper = false;
    public int _ringsFlicked = 0;
    private String _shootingRings = "none";


    public void initialize(LinearOpMode op) {
        flicker = op.hardwareMap.get(Servo.class, "ShooterFlick");
        flicker.setPosition(1.0);
    }

    public void startControl() {
    }

    public void flickThreeRings () {
        if (_ringsFlicked == 0) {
            flicker.setPosition(0.0);
            _ringsFlicked = 3;
        } else {
            flicker.setPosition(1.0);
            _ringsFlicked = 0;
        }
    }

    public void flickOneRing () {
        if (_ringsFlicked == 0) {
            flicker.setPosition(0.68);
            _ringsFlicked = 1;
        } else if (_ringsFlicked == 1) {
            flicker.setPosition(0.33);
            _ringsFlicked = 2;
        } else if (_ringsFlicked == 2) {
            flicker.setPosition(0.0);
            _ringsFlicked = 3;
        } else {
            flicker.setPosition(1.0);
            _ringsFlicked = 0;
        }
    }

    public void readController (Gamepad gamepad) {
        if (gamepad.dpad_right && !_rightDpad) {
            _shootingRings = "three";
        }
        _rightDpad = gamepad.dpad_right;

        if (gamepad.right_bumper && !_rightBumper) {
            _shootingRings = "single";
        }
        _rightBumper = gamepad.right_bumper;
    }

    public void whileOpModeIsActive (LinearOpMode op) {
        this.readController(op.gamepad2);
        if (_shootingRings == "single") {
            flickOneRing();
            _shootingRings = "none";
        } else if (_shootingRings == "three") {
            flickThreeRings();
            _shootingRings = "none";
        }

    }

    public void addTelemetry (Telemetry telemetry) {
        telemetry.addData("Rings Flicker", _ringsFlicked);
    }

    public void stop () {
    }
}