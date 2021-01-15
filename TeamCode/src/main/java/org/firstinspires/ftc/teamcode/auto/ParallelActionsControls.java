package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.wobblegoal.ArmControls;
import org.firstinspires.ftc.teamcode.wobblegoal.GripperControls;


public class ParallelActionsControls {

    private ArmControls armControls = new ArmControls();
    private GripperControls gripperControls = new GripperControls();
    // variables below
    public String _state = "none";

    public void initialize(LinearOpMode op) {
        armControls.initialize(op);
        gripperControls.initialize(op);
    }

    public void startControl() {

    }

    public void whileOpModeIsActive (LinearOpMode op) {

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

    public void ungripWobbleGoal () {
        if (_state == "placeWobble") {
            armControls.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armControls._encoderArm = armControls.arm.getCurrentPosition();

            if (armControls._encoderArm < 5500.0) {
                armControls._powerArm = 1.0;
            } else {
                armControls._powerArm = 0.0;
                _state = "none";
            }

        } else {
            armControls._powerArm = 0.0;
        }

        armControls.arm.setPower(armControls._powerArm);
    }

    public void addTelemetry (Telemetry telemetry) {
        telemetry.addData("Wobble Goal Arm Encoder", "%f", armControls._encoderArm);
    }

    public void stop () {
        armControls.arm.setPower(0.0);
    }
}