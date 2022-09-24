package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@Disabled
public class Claw extends OpMode{
    public Servo ClawServo;
    public static double ServoPositionOpen = 0;
    public static double ServoPositionClose = 0;
    @Override
    public void init(){
        ClawServo = hardwareMap.get(Servo.class, "ClawServo");
    }

    public void loop() {

    }

    public static void Open(){
        ClawServo.setPosition(ServoPositionOpen);
    }
    public static void Close(){
        ClawServo.setPosition(ServoPositionClose);
    }

}
