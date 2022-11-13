package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;


@Autonomous(name="fourbar", group="Training")
public class fourbar extends OpMode {


    Servo fourBarServo;

    @Override
    public void init() {
        fourBarServo = hardwareMap.servo.get("fourBarServo");
        fourBarServo.getController().pwmEnable();




    }

    @Override
    public void loop() {
        fourBarServo.setPosition(0.5);
    }


}
