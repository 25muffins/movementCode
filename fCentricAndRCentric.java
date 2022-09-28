
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.Gamepad;



@Config
@TeleOp(name="fCentricandRCentric", group="Iterative Opmode")
public class fCentricAndRCentric extends OpMode
{
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;

    public static Claw claw = new Claw();
    public static DifferentialArm arm = new DifferentialArm();




    public enum state {
        FCMODE,
        RCMODE
    }
    state currentState = state.FCMODE;

    double slowMode = 1;
    public static double turnSpeed = 0.5;
    public static double slowAmount = 0.4;
    public static double deadZoneAmount = 0.1;
    public static double first = 16;
    public static double second = -16;


    boolean isPressed = true;
    boolean isYPressed = true;

    double offset = 0;
    double setX;
    double setY;
    double botHeading;


    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    BNO055IMU imu;


    public void init(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        claw.init(hardwareMap);
        arm.init(hardwareMap);

        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);

        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("init?", "yes");



    }
    public void init_loop(){

    }
    public void start(){

    }

    public void loop(){



        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double y2 = -gamepad2.left_stick_y;
        double ry2 = gamepad2.right_stick_y;

        findBotHeading();

        //main field centric calculations
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);


        switch (currentState){ //switch between fc and rc with button y
            case FCMODE:
                setX = rotX;
                setY = rotY;
                telemetry.addData("mode", "fieldCentric");
                /*if (currentGamepad1.y && !previousGamepad1.y) { // switching modes
                    currentState = state.RCMODE;
                }*/
                break;
            case RCMODE:
                setY = y;
                setX = x;
                telemetry.addData("mode", "robotCentric");
                /*if (currentGamepad1.y && !previousGamepad1.y) { // switching modes
                    currentState = state.FCMODE;
                }*/
                break;

        }

        //buttons for gamepad 1

        buttonASlow(); //slowmode

        buttonBReset(); //reset FCmode

        buttonXTurnToZero(); //turn to 0

        //buttons for gamepad 2

        buttonYClaw();

        if (gamepad2.dpad_up){
            claw.ServoPositionClose += 0.005;
        }
        if (gamepad2.dpad_down){
            claw.ServoPositionClose -= 0.005;
        }


        if (Math.abs(y) < deadZoneAmount) { //y deadzone
            y = 0;
        }
        if (Math.abs(x) < deadZoneAmount) { //x deadzone
            x = 0;
        }
        if (Math.abs(rx) < deadZoneAmount){ //rx deadzone
            rx = 0;
        }



        //mainMotorMovement
        frontLeftMotor.setPower((setY + setX + rx)*slowMode);
        backLeftMotor.setPower((setY - setX + rx)*slowMode);
        frontRightMotor.setPower((setY - setX - rx)*slowMode);
        backRightMotor.setPower((setY + setX - rx)*slowMode);
        arm.MotorLeft.setPower((y2-ry2)*arm.speed);
        arm.MotorRight.setPower((-y2-ry2)*arm.speed);
        // for right it would be both set to y2


        telemetry.addData("slowmode", slowMode);
        telemetry.addData("botheading", Math.toDegrees(botHeading));
        telemetry.addData("x", currentGamepad1.x);
        telemetry.addData("turnSpeed", turnSpeed);
        telemetry.addData("botheading", botHeading);
        telemetry.addData("offset", offset);




        try {

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad2.copy(gamepad2);
            currentGamepad1.copy(gamepad1);
        } catch (RobotCoreException e) {
            e.printStackTrace();
        }
    }
    public void findBotHeading(){
        double currentBotHeading = -imu.getAngularOrientation().firstAngle;

        botHeading = currentBotHeading - offset;

        //deals with cases like when both offset and heading are negative
        if (Math.toDegrees(botHeading)<-180){
            botHeading = Math.toRadians(180 - ((Math.abs(Math.toDegrees(botHeading))) - 180));
        }
        if (Math.toDegrees(botHeading) >= 180){
            botHeading = Math.toRadians((Math.toDegrees(botHeading) - 180)-180);
        }
    }


    //buttons
    public void buttonXTurnToZero(){
        if (currentGamepad1.x && !previousGamepad1.x){
            while ((botHeading>Math.toRadians(first)||botHeading<Math.toRadians(second))){
                findBotHeading();

                if (botHeading <= Math.toRadians(200) && botHeading >= 0){
                    frontLeftMotor.setPower(-turnSpeed);
                    backLeftMotor.setPower(-turnSpeed);
                    frontRightMotor.setPower(turnSpeed);
                    backRightMotor.setPower(turnSpeed);

                }
                if (botHeading < 0 && botHeading>-180) {
                    frontLeftMotor.setPower(turnSpeed);
                    backLeftMotor.setPower(turnSpeed);
                    frontRightMotor.setPower(-turnSpeed);
                    backRightMotor.setPower(-turnSpeed);
                }

            }

        }
    }
    public void buttonBReset(){
        if (gamepad1.b){
            offset = -imu.getAngularOrientation().firstAngle;
        }
    }
    public void buttonASlow(){
        if (currentGamepad1.a && !previousGamepad1.a) { // slowmode
            if (isPressed) {
                slowMode = slowAmount;
                isPressed = false;

            } else {
                slowMode = 1;
                isPressed = true;

            }
        }
    }
    public void buttonYClaw(){
        if (currentGamepad2.y && !previousGamepad2.y) { //claw
            if (isYPressed){
                claw.Close();
                isYPressed = false;
            }
            else {
                claw.Open();
                isYPressed = true;
            }
        }
    }


}
