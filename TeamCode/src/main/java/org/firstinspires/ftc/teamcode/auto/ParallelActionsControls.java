package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ringtransfer.BoxFlickerEncoderControls;
import org.firstinspires.ftc.teamcode.ringtransfer.BoxSlideTiltControls;
import org.firstinspires.ftc.teamcode.shooter.ShooterPID1Encoder;
import org.firstinspires.ftc.teamcode.wobblegoal.ArmControls;
import org.firstinspires.ftc.teamcode.wobblegoal.GripperControls;


public class ParallelActionsControls {

    private ArmControls armControls = new ArmControls();
    private GripperControls gripperControls = new GripperControls();
    private ShooterPID1Encoder shooterPID1Encoder = new ShooterPID1Encoder();
    private BoxSlideTiltControls boxSlideTiltControls = new BoxSlideTiltControls();
    private BoxFlickerEncoderControls boxFlickerEncoderControls = new BoxFlickerEncoderControls();
    // variables below
    public String _state = "none";

    public void initialize(LinearOpMode op) {
        armControls.initialize(op);
        gripperControls.initialize(op);
        shooterPID1Encoder.initialize(op);
        boxSlideTiltControls.initialize(op);
        boxFlickerEncoderControls.initialize(op);
    }

    public void startControl() {
        shooterPID1Encoder.startControl();
    }

    public void whileOpModeIsActive (LinearOpMode op) {

    }

    public void shooterComponents(LinearOpMode op, double time) {
        //spin up shooter and get box ready to shoot
        if (_state == "prepareShooter") {
            shooterPID1Encoder._targetRPM = 4000;
            shooterPID1Encoder.shooterAuto(op, 3800.0);

            boxSlideTiltControls._currentBoxInShooterPos = true;
            boxSlideTiltControls.whileOpModeIsActive(op, time);
        } else if (_state == "shoot3Rings") {
            shooterPID1Encoder._targetRPM = 4000;
            shooterPID1Encoder.shooterAuto(op, 3800.0);
            boxFlickerEncoderControls.flicker.setPower(-1.0);
            op.sleep(1000);
            _state = "none";
        } else {
            shooterPID1Encoder._targetRPM = 0.0;
            shooterPID1Encoder.stop();
            boxFlickerEncoderControls.stop();
            boxSlideTiltControls._currentBoxInShooterPos = false;
            //boxSlideTiltControls.whileOpModeIsActive(op, time);
        }
    }

    public void resetBox(double currentTime, double lastTime) {
        if (_state == "resetBox") {
            boxSlideTiltControls._lastBoxInShooterPos = true;
            boxSlideTiltControls.autoBoxIntakePos(currentTime, lastTime);
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

            if (armControls._encoderArm < 5500.0) {
                armControls._powerArm = 1.0;
            } else {
                armControls._powerArm = 0.0;
                _state = "none";
            }

        } else if (_state == "raiseWobble") {
            armControls.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armControls._encoderArm = armControls.arm.getCurrentPosition();

            if (armControls._encoderArm > 50.0) {
                armControls._powerArm = -1.0;
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
        } else {
            gripperControls.gripper.setPosition(gripperControls._gripPosClose);
        }
    }

    public void addTelemetry (Telemetry telemetry) {
        telemetry.addData("Wobble Goal Arm Encoder", "%f", armControls._encoderArm);
    }

    public void stop (LinearOpMode op, double time) {
        shooterPID1Encoder._targetRPM = 0.0;
        shooterPID1Encoder.stop();
        armControls.arm.setPower(0.0);
        boxFlickerEncoderControls.stop();
        boxSlideTiltControls._currentBoxInShooterPos = false;
        //boxSlideTiltControls.whileOpModeIsActive(op, time);
    }
}