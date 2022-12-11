package org.firstinspires.ftc.teamcode.autonomous;


import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class fourbarcontroller extends OpMode {


    Servo fourBarServo;
    Servo fourBarServo2;
    CRServo intakeServo;
    public static double pos = 0.5;

    @Override
    public void init(){
        fourBarServo = hardwareMap.servo.get("fourBarServo");
        fourBarServo2 = hardwareMap.servo.get("fourBarServo2");
        fourBarServo.setDirection(Servo.Direction.REVERSE);


        fourBarServo.setPosition(pos * 0.25);
        fourBarServo2.setPosition(pos * 0.25);
        fourBarServo.setPosition(pos * 0.5);
        fourBarServo2.setPosition(pos * 0.5);
        fourBarServo.setPosition(pos * 0.75);
        fourBarServo2.setPosition(pos * 0.75);
        fourBarServo.setPosition(pos);
        fourBarServo2.setPosition(pos);



        fourBarServo.getController().pwmEnable();
        fourBarServo2.getController().pwmEnable();

        telemetry.addData("fourbarservo1", fourBarServo.getPosition());
        telemetry.addData("fourbarservo2", fourBarServo2.getPosition());
        telemetry.addData("fourbarservo1", fourBarServo.getDirection());
        telemetry.addData("fourbarservo2", fourBarServo2.getDirection());
        telemetry.addData("servo1", fourBarServo.getPortNumber());
        telemetry.addData("servo2", fourBarServo2.getPortNumber());


        telemetry.update();





    }
    @Override
    public void loop()
    {
        telemetry.addData("fourbarservo1", fourBarServo.getPosition());
        telemetry.addData("fourbarservo2", fourBarServo2.getPosition());
        telemetry.addData("fourbarservo1", fourBarServo.getDirection());
        telemetry.addData("fourbarservo2", fourBarServo2.getDirection());


        telemetry.update();
    }


}
