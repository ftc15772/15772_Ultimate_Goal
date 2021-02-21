package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shooter.DeflectorControls;


//Testing Sourcetree 1/30/21
@TeleOp(name = "Deflector Teleop", group = "Linear Opmode")
//@Disabled
public class DeflectorTestTeleop extends LinearOpMode {

    double _time = 0.0;

    private DeflectorControls deflectorControls = new DeflectorControls();

    @Override
    public void runOpMode() {

        deflectorControls.initialize(this);

        // Wait for the start button
        waitForStart();

        while(opModeIsActive()) {

            _time = getRuntime();

            deflectorControls.readController(gamepad2);

            deflectorControls.whileOpModeIsActive(this);

            deflectorControls.addTelemetry(telemetry);

            telemetry.addData("Opmode Timer (ms)", _time);
            telemetry.update();
            idle();
        }

    }

}