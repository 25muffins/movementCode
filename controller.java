
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.lang.Thread;

@Config
@TeleOp(name="controller", group="Iterative Opmode")
public class controller extends OpMode
{
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    boolean isPressed = true;
    double slowMode = 1;
    public static double slowAmount = 0.3;
    public static double deadZoneAmount = 0.1;
    public static int SetEncoderPosToZero = 0;
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad1 = new Gamepad();

    public void init(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");

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



    }
    public void init_loop(){
        telemetry.addData("Init?", "yes");
    }
    public void start(){

    }
    public void loop(){
        try {

            previousGamepad1.copy(currentGamepad1);

            currentGamepad1.copy(gamepad1);
        } catch (RobotCoreException e) {
            e.printStackTrace();
        }

        if (currentGamepad1.a && !previousGamepad1.a) { // slowmode
            if (isPressed) {
                slowMode = slowAmount;
                isPressed = false;
            } else {
                slowMode = 1;
                isPressed = true;
            }
        }
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (Math.abs(y) < deadZoneAmount) { //y deadzone
            y = 0;
        }
        if (Math.abs(x) < deadZoneAmount) { //x deadzone
            x = 0;
        }
        if (Math.abs(rx) < deadZoneAmount){ //rx deadzone
            rx = 0;
        }
        frontLeftMotor.setPower((y + x + rx)*slowMode);
        backLeftMotor.setPower((y - x + rx)*slowMode);
        frontRightMotor.setPower((y - x - rx)*slowMode);
        backRightMotor.setPower((y + x - rx)*slowMode);

        if (SetEncoderPosToZero == 1){
            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }



        telemetry.addData("leftY", y);
        telemetry.addData("leftX", x);
        telemetry.addData("rightX", rx);
        telemetry.addData("slowmode", slowMode);
        telemetry.addData("frontLeftEncoderPos", frontLeftMotor.getCurrentPosition());
        telemetry.addData("frontRightEncoderPos", frontRightMotor.getCurrentPosition());
    }

}
