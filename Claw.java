package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
public class Claw{
    public Servo ClawServo = null;

    public static double ServoPositionOpen = 0.42;
    public static double ServoPositionClose = 0.65;
    public void init(HardwareMap hardwareMap){
        ClawServo = hardwareMap.get(Servo.class, "ClawServo");
    }

    public void loop() {

    }
    public void Open(){
        ClawServo.setPosition(ServoPositionOpen);
    }
    public void Close(){
        ClawServo.setPosition(ServoPositionClose);
    }



}
