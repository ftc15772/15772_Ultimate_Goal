package org.firstinspires.ftc.teamcode.wobblegoal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class GripperControls {

    public Servo gripper = null;

    public double _gripPosOpen = 0.35;
    public double _gripPosClose = 0.57;
    private double _gripPosRing = 0.5;
    private double _gripPosCurrent = _gripPosClose;
    private boolean _grippingWobbleGoal = true;
    private boolean _y = false;
    private boolean _b = false;

    public void initialize(LinearOpMode op) {
        gripper = op.hardwareMap.get(Servo.class, "Gripper");

        gripper.setPosition(_gripPosCurrent);
    }

    public void startControl() {
    }

    public void readController (Gamepad gamepad) {
        if (gamepad.y && !_y) {
            _grippingWobbleGoal = !_grippingWobbleGoal;
        }
        _y = gamepad.y;

        /* if (gamepad.b && !_b) {
            _gripPosCurrent = _gripPosRing;
            _grippingWobbleGoal = false;
        }
        _b = gamepad.b; */


        /* if (gamepad.b == true) {
            _gripPosCurrent = _gripPosRing;
        }
        //_a = gamepad.a;

        if (gamepad.a == true) {
            _gripPosCurrent = _gripPosOpen;
        }
        //_rgtBump = gamepad.right_bumper;

        if (gamepad.y == true) {
            _gripPosCurrent = _gripPosClose;
        }
        //_lftBump = gamepad.left_bumper; */
    }

    public void whileOpModeIsActive (LinearOpMode op) {
        this.readController(op.gamepad2);

        /* if ((_grippingWobbleGoal == true) && (_gripPosCurrent != _gripPosRing)) {
            _gripPosCurrent = _gripPosClose;
        } else if ((_grippingWobbleGoal == false) && (_gripPosCurrent != _gripPosRing)) {
            _gripPosCurrent = _gripPosOpen;
        } else {
            _gripPosCurrent = _gripPosRing;
        }*/

        if (_grippingWobbleGoal == true) {
            _gripPosCurrent = _gripPosClose;
        } else if (_grippingWobbleGoal == false) {
            _gripPosCurrent = _gripPosOpen;
        }

        gripper.setPosition(_gripPosCurrent);
    }

    public void addTelemetry (Telemetry telemetry) {

    }

    public void stop () {
    }
}