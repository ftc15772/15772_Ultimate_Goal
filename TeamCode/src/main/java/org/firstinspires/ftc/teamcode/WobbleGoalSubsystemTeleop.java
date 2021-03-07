package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.wobblegoal.ArmControls;
import org.firstinspires.ftc.teamcode.wobblegoal.GripperControls;

@TeleOp(name = "Wobble Goal Subsystem Test", group = "Linear Opmode")
//@Disabled
public class WobbleGoalSubsystemTeleop extends LinearOpMode {

    private ArmControls armControls = new ArmControls();
    private GripperControls gripperControls = new GripperControls();

    @Override
    public void runOpMode() {

        armControls.initialize(this);
        gripperControls.initialize(this);

        // Wait for the start button
        waitForStart();

        gripperControls.startControl();
        armControls.startControl();

        while(opModeIsActive()) {

            armControls.readController(gamepad2);
            gripperControls.readController(gamepad2);

            armControls.whileOpModeIsActive(this);
            gripperControls.whileOpModeIsActive(this);

            armControls.addTelemetry(telemetry);
            telemetry.update();
            idle();
        }

        armControls.stop();

    }

}