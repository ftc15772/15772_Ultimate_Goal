package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePosition;

import java.util.List;

@Autonomous(name = "Red Auto")
public class RedAuto extends LinearOpMode {

    private ParallelActionsControls parallelActionsControls = new ParallelActionsControls();

    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = (8192/5.93687);

    double _permanentX = 116.25; //inches
    double _permanentY = 9; //inches
    double _xFromPermanentPoint;
    double _yFromPermanentPoint;
    //public String _state = "none";
    ElapsedTime Timer;
    double _time = 0.0;
    double _lastTime = 0.0;
    String _ringType = "notDetected"; //"notDetected" to start, then none = "none", one is "one", and four is "four"
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

            //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "FR", rbName = "BR", lfName = "FL", lbName = "BL";
    String verticalLeftEncoderName = "FL", verticalRightEncoderName = "BL", horizontalEncoderName = "BR";
    OdometryGlobalCoordinatePosition globalPositionUpdate;

    private static final String VUFORIA_KEY =
            " Ac/2h37/////AAABmbXAvaZQqkPSlZv4583jp15xBpCuzySKMfid1ppM+8fZbZsGd93ri87TKmjKKCYA64DjBiSRboJvg0eldCw/QzbXtH/gNzdbd90bD226N+MA3p3b4CH+C8Pe+Q2SPV5d4e23K514g/DZGu5JEHHH5kl1guWLfc485PCIGE/wlhIprwSQmGM535rO6oif8Dka9K6zFPkiiSvsj4SoTdVJ9EMPnSYT1LNRUtcWWyN0aCVFJ2cmU2lCAtvS6t7GACGTQAbq+vURBnS0BLwkqgebDbvPPM6y4LOG904dFosYxQsSJw51CCTDNLXlunkQcEzp8DSjH79jiTb6BMwGtpRbFhyGrtSq+ugYlE6uf+C7V913 ";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setClippingMargins(0,250,0,0);
            //                      L T R B
        }

        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        //verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        CameraDevice.getInstance().setFlashTorchMode(true);

        parallelActionsControls.initialize(this);


        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        parallelActionsControls.startControl();

        Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        Timer.reset();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                if (recognition.getLabel() == "Single") {
                    _ringType = "one";
                } else if (recognition.getLabel() == "Quad") {
                    _ringType = "four";
                }
            }
        }
        if (_ringType == "notDetected") {
            goToPosition("none", 106*COUNTS_PER_INCH,15*COUNTS_PER_INCH,0.4,0, 3*COUNTS_PER_INCH);
            right_front.setPower(0);
            right_back.setPower(0);
            left_front.setPower(0);
            left_back.setPower(0);
            while (_ringType == "notDetected") {
                _time = Timer.milliseconds();
                updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        if (recognition.getLabel() == "Single") {
                            _ringType = "one";
                        } else if (recognition.getLabel() == "Quad") {
                            _ringType = "four";
                        }
                    }
                }
                if ((_time > 5000) && (_ringType == "notDetected")) {
                    _ringType = "none";
                }
            }
        }

        goToPosition("placeWobble", 123*COUNTS_PER_INCH, 30*COUNTS_PER_INCH, 0.6, 0, 2*COUNTS_PER_INCH);

        if (_ringType == "four") {
            goToPosition("placeWobble", 123*COUNTS_PER_INCH, 93*COUNTS_PER_INCH, 0.7, 0, 2.5*COUNTS_PER_INCH);
            goToPosition("placeWobble", 121*COUNTS_PER_INCH, 115*COUNTS_PER_INCH, 0.5, 45, 4*COUNTS_PER_INCH);
        } else if (_ringType == "one") {
            goToPosition("placeWobble", 123*COUNTS_PER_INCH, 63*COUNTS_PER_INCH, 0.7, 0, 4*COUNTS_PER_INCH);
            //goToPosition("placeWobble", 98*COUNTS_PER_INCH, 89*COUNTS_PER_INCH, 0.5, 45, 3*COUNTS_PER_INCH);
            goToPosition("placeWobble", 110*COUNTS_PER_INCH, 90*COUNTS_PER_INCH, 0.5, 0, 2.5*COUNTS_PER_INCH);
        } else if (_ringType == "none") {
            goToPosition("placeWobble", 123*COUNTS_PER_INCH, 63*COUNTS_PER_INCH, 0.4, 25, 4*COUNTS_PER_INCH);
            goToPosition("placeWobble", 124*COUNTS_PER_INCH, 67*COUNTS_PER_INCH, 0.4, 45, 2.5*COUNTS_PER_INCH);
        }
        right_front.setPower(0);
        right_back.setPower(0);
        left_front.setPower(0);
        left_back.setPower(0);
        parallelActionsControls._state = "ungripWobble";
        parallelActionsControls.wobbleGoal();
        sleep(300);
        if (_ringType == "four") {
            goToPosition("prepareShooter", 112*COUNTS_PER_INCH, 106*COUNTS_PER_INCH, 0.4, 45, 3*COUNTS_PER_INCH);
        } else if (_ringType == "one") {
            //goToPosition("prepareShooter", 94*COUNTS_PER_INCH, 83*COUNTS_PER_INCH, 0.4, 45, 2*COUNTS_PER_INCH);
            goToPosition("prepareShooter", 110*COUNTS_PER_INCH, 85*COUNTS_PER_INCH, 0.4, 0, 2*COUNTS_PER_INCH);
        } else if (_ringType == "none") {
            goToPosition("prepareShooter", 120*COUNTS_PER_INCH, 63*COUNTS_PER_INCH, 0.3, 45, 2*COUNTS_PER_INCH);
        }
        goToPosition("prepareShooter", 105*COUNTS_PER_INCH, 65*COUNTS_PER_INCH, 0.6, 8, 2*COUNTS_PER_INCH);


        right_front.setPower(0);
        right_back.setPower(0);
        left_front.setPower(0);
        left_back.setPower(0);
        parallelActionsControls._state = "shoot3Rings";
        while (parallelActionsControls._state != "none") {
            _time = Timer.seconds();
            parallelActionsControls.shooterComponents(this, _time, telemetry);
            telemetry.update();
        }
        parallelActionsControls.stop(this, Timer.milliseconds());

        goToPosition("raiseWobble", 105*COUNTS_PER_INCH, 80*COUNTS_PER_INCH, 0.3, 0, 1.5*COUNTS_PER_INCH);


        right_front.setPower(0);
        right_back.setPower(0);
        left_front.setPower(0);
        left_back.setPower(0);
        _time = Timer.milliseconds();
        parallelActionsControls._state = "resetBox";
        _lastTime = _time = Timer.milliseconds();
        while (parallelActionsControls._state == "resetBox") {
            _time = Timer.milliseconds();
            parallelActionsControls.resetBox(_time, _lastTime);
        }
        sleep(1000);



        /* while(opModeIsActive()){
            telemetry.addData("Ring Type", _ringType);

            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", _xFromPermanentPoint);
            telemetry.addData("Y Position", _yFromPermanentPoint);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        } */

        _time = Timer.milliseconds();
        parallelActionsControls.stop(this, Timer.milliseconds());
        //Stop the thread
        globalPositionUpdate.stop();

    }

    public void goToPosition(String state, double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError){
        _xFromPermanentPoint = (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH) + _permanentX;
        _yFromPermanentPoint = (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH) + _permanentY;

        double distanceToXTarget = targetXPosition - (_xFromPermanentPoint * COUNTS_PER_INCH);
        double distanceToYTarget = targetYPosition - (_yFromPermanentPoint * COUNTS_PER_INCH);

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        while (opModeIsActive() && distance > allowableDistanceError){

            _xFromPermanentPoint = (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH) + _permanentX;
            _yFromPermanentPoint = (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH) + _permanentY;

            distance = Math.hypot(distanceToXTarget, distanceToYTarget);

            distanceToXTarget = targetXPosition - (_xFromPermanentPoint * COUNTS_PER_INCH);
            distanceToYTarget = targetYPosition - (_yFromPermanentPoint * COUNTS_PER_INCH);

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget,distanceToYTarget));

            double robot_movement_x_component = calculateX(robotMovementAngle, robotPower);
            double robot_movement_y_component = calculateY(robotMovementAngle, robotPower);
            double pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();

            double _magnitude = Math.sqrt((robot_movement_x_component*robot_movement_x_component)+(robot_movement_y_component*robot_movement_y_component));
            double _theta = Math.atan2(robot_movement_x_component, robot_movement_y_component);
            double quarterPi = Math.PI / 4;

            double _turnRate = Math.toRadians(pivotCorrection) * robotPower;

            double _powerFL = _magnitude * Math.sin(_theta + quarterPi) + _turnRate;
            double _powerFR = _magnitude * Math.cos(_theta + quarterPi) - _turnRate;
            double _powerBL = _magnitude * Math.cos(_theta + quarterPi) + _turnRate;
            double _powerBR = _magnitude * Math.sin(_theta + quarterPi) - _turnRate;


            double scale = Math.max(Math.max(Math.abs(_powerFL), Math.abs(_powerFR)),
                    Math.max(Math.abs(_powerBL), Math.abs(_powerBR)));

            if (scale > 1.0) {
                _powerFL /= scale;
                _powerFR /= scale;
                _powerBL /= scale;
                _powerBR /= scale;
            }

            left_front.setPower(-_powerFL);
            right_front.setPower(_powerFR);
            left_back.setPower(-_powerBL);
            right_back.setPower(-_powerBR);

            _time = Timer.milliseconds();
            parallelActionsControls._state = state;
            parallelActionsControls.wobbleGoal();
            parallelActionsControls.shooterComponents(this, _time, telemetry);

            //Display Global (x, y, theta) coordinates
            telemetry.addData("Ring Type", _ringType);
            telemetry.addData("X Position", _xFromPermanentPoint);
            telemetry.addData("Y Position", _yFromPermanentPoint);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.update();
        }
    }

    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        //left_back.setDirection(DcMotorSimple.Direction.REVERSE);
        //right_front.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}