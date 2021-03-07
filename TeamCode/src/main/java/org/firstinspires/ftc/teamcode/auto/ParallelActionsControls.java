package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ringtransfer.BoxFlickerPositionModeControls;
import org.firstinspires.ftc.teamcode.ringtransfer.BoxSlideTiltControls;
import org.firstinspires.ftc.teamcode.shooter.DeflectorControls;
import org.firstinspires.ftc.teamcode.shooter.ShooterPID1Encoder;
import org.firstinspires.ftc.teamcode.wobblegoal.ArmControls;
import org.firstinspires.ftc.teamcode.wobblegoal.GripperControls;


public class ParallelActionsControls {

    private ArmControls armControls = new ArmControls();
    private GripperControls gripperControls = new GripperControls();
    public ShooterPID1Encoder shooterPID1Encoder = new ShooterPID1Encoder();
    private BoxSlideTiltControls boxSlideTiltControls = new BoxSlideTiltControls();
    public BoxFlickerPositionModeControls boxFlickerPositionModeControls = new BoxFlickerPositionModeControls();
    private DeflectorControls deflectorControls = new DeflectorControls();
    // variables below
    public String _state = "none";
    public double _lastTime;

    public void initialize(LinearOpMode op) {
        armControls.initialize(op);
        gripperControls.initialize(op);
        shooterPID1Encoder.initialize(op);
        boxSlideTiltControls.initialize(op);
        boxFlickerPositionModeControls.initialize(op);
        deflectorControls.initialize(op);
        deflectorControls.deflector.setPosition(deflectorControls._highGoalPosAuto);
    }

    public void startControl() {
        shooterPID1Encoder.startControl();
    }

    public void shooterComponents(LinearOpMode op, double time, Telemetry telemetry) {
        double _time = time;
        //spin up shooter and get box ready to shoot
        if (_state == "prepareShooter") {
            shooterPID1Encoder.shooterAuto2(op, 3600);
            shooterPID1Encoder.addTelemetry(telemetry);

            boxSlideTiltControls._currentBoxInShooterPos = true;
            boxSlideTiltControls.whileOpModeIsActive(op, _time / 1000);
        } else if (_state == "shoot3Rings") {
            /*
            if ((_time - _lastTime) <= 1.0) {
                shooterPID1Encoder.shooterAuto2(op, 3600);
                shooterPID1Encoder.addTelemetry(telemetry);
            } else if (((_time - _lastTime) <= 2.0) && ((_time - _lastTime) > 1.0)) {
                shooterPID1Encoder.shooterAuto2(op, 3600);
                boxFlickerPositionModeControls.flickThreeRings();
                shooterPID1Encoder.addTelemetry(telemetry);
            } else {
                _state = "none";
            }
            */


            /*
            boxFlickerPositionModeControls.flickOneRing();
            op.sleep(300);
            boxFlickerPositionModeControls.flickOneRing();
            op.sleep(300);
            boxFlickerPositionModeControls.flickOneRing();
            op.sleep(300);
            _state = "none";
            */

            shooterPID1Encoder.shooterAuto2(op, 3600);

            if (((_time - _lastTime) >= 1.0) && (boxFlickerPositionModeControls._ringsFlicked == 2)) {
                boxFlickerPositionModeControls.flickOneRing();
            } else if (((_time - _lastTime) >= 0.5) && (boxFlickerPositionModeControls._ringsFlicked == 1)) {
                boxFlickerPositionModeControls.flickOneRing();
            } else if (((_time - _lastTime) >= 0) && (boxFlickerPositionModeControls._ringsFlicked == 0)) {
                boxFlickerPositionModeControls.flickOneRing();
            }
            shooterPID1Encoder.addTelemetry(telemetry);
            if ((_time - _lastTime) >= 1.5) {
                _state = "none";

            }


        } else {
            boxSlideTiltControls._currentBoxInShooterPos = false;
        }
    }

    public void resetBox(LinearOpMode op, double time){ //double currentTime, double lastTime) {
        if (_state == "resetBox") {

            /* armControls.arm.setPower(0);
            boxSlideTiltControls._lastBoxInShooterPos = true;
            boxSlideTiltControls.autoBoxIntakePos(currentTime, lastTime);
            if (boxSlideTiltControls._lastBoxInShooterPos == false) {
                _state = "none";
            } */

            double _time = time;
            armControls.arm.setPower(0);

            /* if (boxFlickerPositionModeControls._ringsFlicked == 3) {
                boxFlickerPositionModeControls._ringsFlicked = 0;
                boxFlickerPositionModeControls.flicker.setPosition(1.0);
            } else if ((boxFlickerPositionModeControls._ringsFlicked == 0) || ((_time - _lastTime) >= 0.25)) {
                boxSlideTiltControls._currentBoxInShooterPos = false;
                boxSlideTiltControls._lastBoxInShooterPos = true;
                boxSlideTiltControls.whileOpModeIsActive(op, _time / 1000);
            }
             */
            boxSlideTiltControls._currentBoxInShooterPos = false;
            boxSlideTiltControls._lastBoxInShooterPos = true;
            boxSlideTiltControls.whileOpModeIsActive(op, _time / 1000);
            if (boxSlideTiltControls._lastBoxInShooterPos == false) {
                _state = "none";
            }
        }
    }

    public void wobbleGoal() {
        // lower or raise arm
        if (_state == "placeWobble") {
            armControls.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armControls._encoderArm = armControls.arm.getCurrentPosition();

            if (armControls._encoderArm < 3700) {
                armControls._powerArm = 1.0;
            } else {
                armControls._powerArm = 0.0;
                _state = "none";
            }

        } else if ((_state == "raiseWobble") || (_state == "prepareShooter")) {
            armControls.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armControls._encoderArm = armControls.arm.getCurrentPosition();

            if (armControls._encoderArm > 1800) {
                armControls._powerArm = -1.0;
            } else {
                armControls._powerArm = 0.0;
            }

        } else if (_state == "grabWobble") {
            armControls.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armControls._encoderArm = armControls.arm.getCurrentPosition();

            if (armControls._encoderArm < 4500) {
                armControls._powerArm = 1.0;
            } else {
                armControls._powerArm = 0.0;
                _state = "none";
            }

        } else {
            armControls._powerArm = 0.0;
        }

        armControls.arm.setPower(armControls._powerArm);

        // ungrip wobble goal
        if (_state == "ungripWobble") {
            gripperControls.gripper.setPosition(gripperControls._gripPosOpen);
        }

        if (_state == "gripWobble") {
            gripperControls.gripper.setPosition(gripperControls._gripPosClose);
        }
    }

    public void addTelemetry (Telemetry telemetry) {
        shooterPID1Encoder.addTelemetry(telemetry);
    }

    public void stop () {
        shooterPID1Encoder._targetRPM = 0.0;
        shooterPID1Encoder.stop();
        armControls.arm.setPower(0.0);
        boxSlideTiltControls._currentBoxInShooterPos = false;
        //boxSlideTiltControls.whileOpModeIsActive(op, time);
    }
}