
package org.firstinspires.ftc.teamcode;

import android.provider.Settings;

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
@TeleOp(name="octacanum", group="Iterative Opmode")
public class octacanum extends OpMode
{
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor linearMotor;

    double slowMode = 1;
    public static double slowAmount = 0.3;
    double offset = 0;
    public static double deadZoneAmount = 0.1;

    boolean isAPressed = true;
    boolean isYPressed = true;
    boolean isXPressed = true;

    boolean GlobalPastFC;
    boolean GlobalPastRC;
    boolean FCmode = true;
    boolean RCmode = false;
    boolean RHmode = false;

    double setX;
    double setY;

    public static int TargetPosition = 0;
    public static double TargetSpeed = 0;
    public static double controllerSensitivity = 0.1;


    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad1 = new Gamepad();

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
        linearMotor = hardwareMap.get(DcMotor.class, "linearMotor");

        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        linearMotor.setPower(0);

        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);





    }
    public void init_loop(){
        telemetry.addData("Init?", "yes");
    }
    public void start(){

    }
    public void loop(){
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double originalBotHeading = -imu.getAngularOrientation().firstAngle;
        double botHeading = originalBotHeading - offset;
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        if (FCmode){
            setX = rotX;
            setY = rotY;
            telemetry.addData("mode", "fieldCentric");
        }
        else if(RCmode){
            setY = y;
            setX = x;
            telemetry.addData("mode", "robotCentric");
        }
        else if(RHmode){
            setY = y;
            setX = 0;
            telemetry.addData("mode", "rhinoMode");
        }

        if (currentGamepad1.a && !previousGamepad1.a) { // slowmode
            if (isAPressed) {
                slowMode = slowAmount;
                isAPressed = false;

            } else {
                slowMode = 1;
                isAPressed = true;

            }
        }



        if (gamepad1.b){
            offset = -imu.getAngularOrientation().firstAngle;
        }

        if (currentGamepad1.y && !previousGamepad1.y) { // switching modes

            if (isYPressed) {
                FCmode = false;
                RCmode = true;
                RHmode = false;
                isYPressed = false;
            } else {
                FCmode = true;
                RCmode = false;
                RHmode = false;
                isYPressed = true;

            }
        }
        if (currentGamepad1.x && !previousGamepad1.x){ // switch drives

            if (isXPressed) {
                //TICKS_PER_REV = 537.6;

                linearMotor.setTargetPosition(TargetPosition);

                linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearMotor.setPower(TargetSpeed);


                GlobalPastFC = FCmode;
                GlobalPastRC = RCmode;

                isXPressed = false;
                FCmode = false;
                RCmode = false;
                RHmode = true;



            } else {
                linearMotor.setTargetPosition(0);

                linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearMotor.setPower(0);



                isXPressed = true;
                FCmode = GlobalPastFC;
                RCmode = GlobalPastRC;
                RHmode = false;

            }
        }
            linearMotor.setPower(-gamepad1.right_stick_y * controllerSensitivity);
            linearMotor.setPower(gamepad1.right_stick_y * controllerSensitivity);



        if (Math.abs(y) < deadZoneAmount) { //y deadzone
            y = 0;
        }
        if (Math.abs(x) < deadZoneAmount) { //x deadzone
            x = 0;
        }
        if (Math.abs(rx) < deadZoneAmount){ //rx deadzone
            rx = 0;
        }

        frontLeftMotor.setPower((setY + setX + rx)*slowMode);
        backLeftMotor.setPower((setY - setX + rx)*slowMode);
        frontRightMotor.setPower((setY - setX - rx)*slowMode);
        backRightMotor.setPower((setY + setX - rx)*slowMode);




        telemetry.addData("slowmode", slowMode);
        telemetry.addData("Y", y);
        telemetry.addData("X", x);
        telemetry.addData("EncoderTicksPosition", linearMotor.getCurrentPosition());
        telemetry.addData("TargetPosition", TargetPosition);



        try {

            previousGamepad1.copy(currentGamepad1);

            currentGamepad1.copy(gamepad1);
        } catch (RobotCoreException e) {
            e.printStackTrace();
        }
    }

}
