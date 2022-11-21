package org.firstinspires.ftc.teamcode.autonomous;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name="fourbarcontroller", group="Training")
public class fourbarcontroller extends OpMode {


    Servo fourBarServo;
    Servo fourBarServo2;
    public static double pos = 0;

    @Override
    public void init() {
        fourBarServo = hardwareMap.servo.get("fourBarServo");
        fourBarServo2 = hardwareMap.servo.get("fourBarServo2");
        fourBarServo.setDirection(Servo.Direction.REVERSE);
        fourBarServo.setPosition(pos);
        fourBarServo2.setPosition(pos);


        fourBarServo.getController().pwmEnable();
        fourBarServo2.getController().pwmEnable();

        telemetry.addData("fourbarservo1", fourBarServo.getPosition());
        telemetry.addData("fourbarservo2", fourBarServo2.getPosition());
        telemetry.addData("fourbarservo1", fourBarServo.getDirection());
        telemetry.addData("fourbarservo2", fourBarServo2.getDirection());


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

    public void servoPos() {
        if (gamepad1.circle) {

        }
        else if (gamepad1.cross) {
            fourBarServo2.setPosition(pos);
        }
        else if (gamepad1.triangle) {
            fourBarServo.setDirection((Servo.Direction.REVERSE));
        }
        else if (gamepad1.square) {
            fourBarServo2.setDirection((Servo.Direction.REVERSE));
        }
    }


}
