package org.firstinspires.ftc.teamcode.testteleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.odometry.OdometryControls;

@TeleOp(name = "Odometry Test Teleop", group = "Linear Opmode")
@Disabled
public class OdometryTestTeleop extends LinearOpMode {

    double _time = 0.0;

    //private MecanumDrivebase mecanumDrivebase = new MecanumDrivebase();
    //private GyroSensor gyroSensor = new GyroSensor();
    private OdometryControls odometryControls = new OdometryControls();

    @Override
    public void runOpMode() {

        //mecanumDrivebase.initialize(this);
        //gyroSensor.initialize(this);
        odometryControls.initialize(this);

        // Wait for the start button
        waitForStart();

        //mecanumDrivebase.startControl();
        //gyroSensor.startControl();
        odometryControls.startControl(telemetry, this);

        while(opModeIsActive()) {

            _time = getRuntime();

            //mecanumDrivebase.readController(gamepad1);

            //gyroSensor.updateAngles(this);

            //mecanumDrivebase.setGyroAngle(gyroSensor.getDirection());

            //mecanumDrivebase.whileOpModeIsActive(this);
            odometryControls.whileOpModeIsActive(this);

            //mecanumDrivebase.addTelemetry(telemetry);
            odometryControls.addTelemetry(telemetry);
            telemetry.addData("Opmode Timer (ms)", _time);
            telemetry.update();
            idle();
        }

        //mecanumDrivebase.stop();
        odometryControls.stop();
    }

}