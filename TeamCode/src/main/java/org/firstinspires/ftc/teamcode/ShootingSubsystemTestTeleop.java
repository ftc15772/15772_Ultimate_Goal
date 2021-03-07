package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ringtransfer.BoxFlickerPositionModeControls;
import org.firstinspires.ftc.teamcode.ringtransfer.BoxSlideTiltControls;
import org.firstinspires.ftc.teamcode.shooter.DeflectorControls;
import org.firstinspires.ftc.teamcode.shooter.ShooterPID1Encoder;


@TeleOp(name = "Shooting Subsystem Test", group = "Linear Opmode")
//@Disabled
public class ShootingSubsystemTestTeleop extends LinearOpMode {

    double _time = 0.0;
    public boolean _dpadUp = false;
    public boolean _dpadDown = false;
    public String shooterState = "off";

    private ShooterPID1Encoder shooterPID1Encoder = new ShooterPID1Encoder();
    private BoxFlickerPositionModeControls boxFlickerPositionModeControls = new BoxFlickerPositionModeControls();
    private BoxSlideTiltControls boxSlideTiltControls = new BoxSlideTiltControls();
    private DeflectorControls deflectorControls = new DeflectorControls();

    @Override
    public void runOpMode() {

        shooterPID1Encoder.initialize(this);
        boxFlickerPositionModeControls.initialize(this);
        boxSlideTiltControls.initialize(this);
        deflectorControls.initialize(this);

        // Wait for the start button
        waitForStart();

        shooterPID1Encoder.startControl();
        boxFlickerPositionModeControls.startControl();

        while(opModeIsActive()) {

            _time = getRuntime();

            /* if (!_dpadUp && gamepad2.dpad_up) {
                shooterState = "on";
            } else if (!_dpadDown && gamepad2.dpad_down) {
                shooterState = "off";
            }
            _dpadUp = gamepad2.dpad_up;
            _dpadDown = gamepad2.dpad_down;

            if (shooterState == "on") {
                shooterPID1Encoder.shooterAuto2(this,3800);
            } else {
                shooterPID1Encoder.shooterAuto2(this,0);
                shooterPID1Encoder.stop();
            }
             */



            shooterPID1Encoder.readController(gamepad2);
            boxFlickerPositionModeControls.readController(gamepad2);
            boxSlideTiltControls.readController(gamepad2);
            deflectorControls.readController(gamepad2);

            shooterPID1Encoder.whileOpModeIsActive(this);
            boxFlickerPositionModeControls.whileOpModeIsActive(this);
            boxSlideTiltControls.whileOpModeIsActive(this, _time);
            deflectorControls.whileOpModeIsActive(this);

            shooterPID1Encoder.addTelemetry(telemetry);
            boxFlickerPositionModeControls.addTelemetry(telemetry);
            deflectorControls.addTelemetry(telemetry);

            telemetry.addData("Opmode Timer (ms)", _time);
            telemetry.update();
            idle();
        }

        shooterPID1Encoder.stop();
        boxFlickerPositionModeControls.stop();

    }

}