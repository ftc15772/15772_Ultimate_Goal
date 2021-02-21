package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.intake.ComplicatedSnowplowControls;
import org.firstinspires.ftc.teamcode.intake.IntakeControls;
import org.firstinspires.ftc.teamcode.ringtransfer.BoxFlickerEncoderControlsNew;
import org.firstinspires.ftc.teamcode.ringtransfer.BoxSlideTiltControls;
import org.firstinspires.ftc.teamcode.shooter.DeflectorControls;
import org.firstinspires.ftc.teamcode.shooter.ShooterPID1Encoder;
import org.firstinspires.ftc.teamcode.wobblegoal.ArmControls;
import org.firstinspires.ftc.teamcode.wobblegoal.GripperControls;


@TeleOp(name = "Full Robot Teleop (NEW)", group = "Linear Opmode")
//@Disabled
public class FullRobotTeleop3 extends LinearOpMode {

    double _time = 0.0;

    private MecanumDrivebase mecanumDrivebase = new MecanumDrivebase();
    //private GyroSensor gyroSensor = new GyroSensor();
    //private OdometryControls odometryControls = new OdometryControls();
    private ShooterPID1Encoder shooterPID1Encoder = new ShooterPID1Encoder();
    private BoxFlickerEncoderControlsNew flickerControls = new BoxFlickerEncoderControlsNew();
    private ArmControls armControls = new ArmControls();
    private IntakeControls intakeControls = new IntakeControls();
    private ComplicatedSnowplowControls plowControls = new ComplicatedSnowplowControls();
    private GripperControls gripperControls = new GripperControls();
    private BoxSlideTiltControls boxSlideTiltControls = new BoxSlideTiltControls();
    private DeflectorControls deflectorControls = new DeflectorControls();
    //private TimerControls timer = new TimerControls();

    @Override
    public void runOpMode() {

        mecanumDrivebase.initialize(this);
        //gyroSensor.initialize(this);
        //odometryControls.initialize(this);
        shooterPID1Encoder.initialize(this);
        flickerControls.initialize(this);
        plowControls.initialize(this);
        armControls.initialize(this);
        intakeControls.initialize(this);
        gripperControls.initialize(this);
        boxSlideTiltControls.initialize(this);
        deflectorControls.initialize(this);
        //boxSliderControls.initialize(this);
        //boxTilterControls.initialize(this);
        //timer.initialize(this);

        // Wait for the start button
        waitForStart();

        mecanumDrivebase.startControl();
        //gyroSensor.startControl();
        //odometryControls.startControl(telemetry, this);
        shooterPID1Encoder.startControl();
        flickerControls.startControl();
        gripperControls.startControl();
        intakeControls.startControl();
        //armControls.startControl();
        //boxSliderControls.startControl();
        //boxTilterControls.startControl();
        //timer.startControl();
        //plowControls.startControl();

        while(opModeIsActive()) {

            _time = getRuntime();

            mecanumDrivebase.readController(gamepad1);
            //odometryControls.readController(gamepad1);
            intakeControls.readController(gamepad1);


            shooterPID1Encoder.readController(gamepad2);
            flickerControls.readController(gamepad2);
            boxSlideTiltControls.readController(gamepad2);
            armControls.readController(gamepad2);
            plowControls.readController(gamepad2);
            gripperControls.readController(gamepad2);
            deflectorControls.readController(gamepad2);
            //boxSliderControls.readController(gamepad2);
            //boxTilterControls.readController(gamepad2);

            //gyroSensor.updateAngles(this);

            //mecanumDrivebase.setGyroAngle(gyroSensor.getDirection());

            mecanumDrivebase.whileOpModeIsActive(this);
            //odometryControls.whileOpModeIsActive(this);
            shooterPID1Encoder.whileOpModeIsActive(this);
            flickerControls.whileOpModeIsActive(this, _time);
            boxSlideTiltControls.whileOpModeIsActive(this, _time);
            intakeControls.whileOpModeIsActive(this);
            armControls.whileOpModeIsActive(this);
            plowControls.whileOpModeIsActive(this, _time);
            gripperControls.whileOpModeIsActive(this);
            deflectorControls.whileOpModeIsActive(this);
            //plowControls.whileOpModeIsActive(this);
            //boxSliderControls.whileOpModeIsActive(this, boxTilterControls._tiltingDown);
            //boxTilterControls.whileOpModeIsActive(this, boxSliderControls._sliderIn);

            //mecanumDrivebase.addTelemetry(telemetry);
            shooterPID1Encoder.addTelemetry(telemetry);
            flickerControls.addTelemetry(telemetry, _time);
            deflectorControls.addTelemetry(telemetry);
            //armControls.addTelemetry(telemetry);
            //intakeControls.addTelemetry(telemetry);
            //plowControls.addTelemetry(telemetry);
            //odometryControls.addTelemetry(telemetry);
            //timer.addTelemetry(telemetry);

            telemetry.addData("Opmode Timer (ms)", _time);
            telemetry.update();
            idle();
        }

        mecanumDrivebase.stop();
        shooterPID1Encoder.stop();
        //odometryControls.stop();
        flickerControls.stop();
        intakeControls.stop();
        armControls.stop();

    }

}