package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="builtDiffy")
@Config
public class DifferentialSlide extends OpMode{
    public DcMotor MotorLeft;
    public DcMotor MotorRight;
    public static double speed = 0.7;
    @Override
    public void init(){
        MotorLeft= hardwareMap.get(DcMotor.class, "MotorLeft");
        MotorRight = hardwareMap.get(DcMotor.class, "MotorRight");

        MotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MotorLeft.setPower(0);
        MotorRight.setPower(0);


        MotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        MotorLeft.setDirection(DcMotor.Direction.FORWARD);
        MotorRight.setDirection(DcMotor.Direction.FORWARD);
    }

    public void loop(){
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;

        MotorLeft.setPower(y*speed);
        MotorRight.setPower(x*speed);

    }

}
