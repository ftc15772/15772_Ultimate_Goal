package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ringtransfer.BoxFlickerEncoderControlsNew;


@TeleOp(name = "Flicker Test Teleop", group = "Linear Opmode")
//@Disabled
public class FlickerTestTeleop extends LinearOpMode {

    double _time = 0.0;

    private BoxFlickerEncoderControlsNew flickerControls = new BoxFlickerEncoderControlsNew();

    @Override
    public void runOpMode() {

        flickerControls.initialize(this);

        // Wait for the start button
        waitForStart();

        flickerControls.startControl();

        while(opModeIsActive()) {

            _time = getRuntime();

            flickerControls.readController(gamepad2);

            flickerControls.whileOpModeIsActive(this, _time);


            flickerControls.addTelemetry(telemetry);
            telemetry.addData("Opmode Timer (ms)", _time);
            telemetry.update();
            idle();
        }

        flickerControls.stop();

    }

}