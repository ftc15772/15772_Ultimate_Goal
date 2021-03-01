package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.auto.ParallelActionsControls;

@Autonomous(name = "My Odometry Vector Mecanum")
public class MyOdometryVectorMecanum extends LinearOpMode {

    private ParallelActionsControls parallelActionsControls = new ParallelActionsControls();

    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = (8192/5.93687);

    double _permanentX = 8; //inches
    double _permanentY = 6; //inches
    double _xFromPermanentPoint;
    double _yFromPermanentPoint;
    double _time;
    //public String _state = "none";

            //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "FR", rbName = "BR", lfName = "FL", lbName = "BL";
    String verticalLeftEncoderName = "FL", verticalRightEncoderName = "BL", horizontalEncoderName = "BR";
    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        //verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        parallelActionsControls.initialize(this);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        goToPosition("placeWobble", 12*COUNTS_PER_INCH, 32*COUNTS_PER_INCH, 0.35, 45, 1.5*COUNTS_PER_INCH);
        parallelActionsControls._state = "ungripWobble";
        parallelActionsControls.wobbleGoal();
        sleep(250);
        goToPosition("raiseWobble", 8*COUNTS_PER_INCH, 29*COUNTS_PER_INCH, 0.3, 45, 1.5*COUNTS_PER_INCH);
        goToPosition("raiseWobble", 24*COUNTS_PER_INCH, 8*COUNTS_PER_INCH, 0.35, 0, 1.5*COUNTS_PER_INCH);


        parallelActionsControls.stop();
        right_front.setPower(0);
        right_back.setPower(0);
        left_front.setPower(0);
        left_back.setPower(0);

        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", _xFromPermanentPoint);
            telemetry.addData("Y Position", _yFromPermanentPoint);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        parallelActionsControls.stop();
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

            parallelActionsControls._state = state;
            parallelActionsControls.wobbleGoal();

            //Display Global (x, y, theta) coordinates
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
}