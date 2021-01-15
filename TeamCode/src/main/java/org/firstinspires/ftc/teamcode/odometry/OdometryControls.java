package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OdometryControls {

    double _xCoordinate;
    double _yCoordinate;
    double _orientation;

    //Odometry encoder wheels
    DcMotor verticalRight, verticalLeft, horizontal;

    //The amount of encoder ticks for each inch the robot moves. This will change for each robot and needs to be changed here
    final double COUNTS_PER_INCH = (8192/5.93687);

    double _permanentX = 117; //inches
    double _permanentY = 9; //inches
    double _xFromPermanentPoint;
    double _yFromPermanentPoint;
    boolean _back;
    boolean _cornerResetBR = false;
    OdometryGlobalCoordinatePosition _globalPositionUpdate = null;
    Thread _positionThread = null;

    //Hardware map names for the encoder wheels. Again, these will change for each robot and need to be updated below
    //String rfName = "FR", rbName = "BR", lfName = "FL", lbName = "BL";
    String verticalLeftEncoderName = "FL", verticalRightEncoderName = "BL", horizontalEncoderName = "BR";

    public void initialize(LinearOpMode op) {

        verticalLeft = op.hardwareMap.dcMotor.get(verticalLeftEncoderName);
        verticalRight = op.hardwareMap.dcMotor.get(verticalRightEncoderName);
        horizontal = op.hardwareMap.dcMotor.get(horizontalEncoderName);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void startControl(Telemetry telemetry, LinearOpMode op) {
        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions\
        _globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        _positionThread = new Thread(_globalPositionUpdate);
        _positionThread.start();

        _globalPositionUpdate.reverseLeftEncoder();
        _globalPositionUpdate.reverseRightEncoder();
        _globalPositionUpdate.reverseNormalEncoder();

        telemetry.addData("Thread Status", "Started");
        telemetry.update();
        op.sleep(250);
    }


    public void returnCoordinates () {
        resetRobotEncoder();
        _xCoordinate = _globalPositionUpdate.returnXCoordinate();
        _yCoordinate = _globalPositionUpdate.returnYCoordinate();
        _orientation = _globalPositionUpdate.returnOrientation();

        _xFromPermanentPoint = (_xCoordinate / COUNTS_PER_INCH) + _permanentX;
        _yFromPermanentPoint = (_yCoordinate / COUNTS_PER_INCH) + _permanentY;
    }


    public void resetRobotEncoder () {
        if (_cornerResetBR == true) {
            _xCoordinate = 15 * COUNTS_PER_INCH;
            _yCoordinate = 0 * COUNTS_PER_INCH;
            _xFromPermanentPoint = 132;
            _yFromPermanentPoint = 9;
            _orientation = 0;
            _cornerResetBR = false;
        }
    }

    public void readController (Gamepad gamepad) {
        if (gamepad.back == true) {
            _cornerResetBR = true;
            resetRobotEncoder();
        }
    }

    public void whileOpModeIsActive (LinearOpMode op) {
        returnCoordinates();
    }

    public void addTelemetry (Telemetry telemetry) {
        returnCoordinates();
        //Display Global (x, y, theta) coordinates
        telemetry.addData("X Position", _xFromPermanentPoint);
        telemetry.addData("Y Position", _yFromPermanentPoint);
        telemetry.addData("Orientation (Degrees)", _orientation);
        //telemetry.addData("Other X Pos", _xCoordinate / COUNTS_PER_INCH);
        //telemetry.addData("Other Y Pos", _yCoordinate / COUNTS_PER_INCH);
        telemetry.addData("Vertical Left Encoder Position", verticalLeft.getCurrentPosition());
        telemetry.addData("Vertical Right Encoder Position", verticalRight.getCurrentPosition());
        telemetry.addData("Horizontal Encoder Position", horizontal.getCurrentPosition());
        telemetry.update();
    }

    public void stop () {
        _globalPositionUpdate.stop();
    }
}