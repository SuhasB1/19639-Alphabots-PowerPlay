package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@Autonomous(name="fourbaraxon")
public class fourbaraxon extends LinearOpMode {

    ServoImplEx leftFourBar;
    ServoImplEx rightFourBar;
    Servo clawServo;
    Servo turretServo;
    public static double turretPos;
    public static double clawPos;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFourBar = hardwareMap.get(ServoImplEx.class,"leftFourBar");
        rightFourBar = hardwareMap.get(ServoImplEx.class,"rightFourBar");
        clawServo = hardwareMap.get(ServoImplEx.class,"clawServo");
        turretServo = hardwareMap.get(ServoImplEx.class,"turretServo");

        leftFourBar.setDirection(Servo.Direction.REVERSE);

        leftFourBar.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightFourBar.setPwmRange(new PwmControl.PwmRange(500, 2500));

        /*
        Claw facing backwards position: 0.15
        Claw facing outwards position: 0.825

        Claw open: 0.4
        Claw close: 0.62
         */





        waitForStart();

        while(opModeIsActive())
        {
            leftFourBar.setPosition(0.75);
            rightFourBar.setPosition(0.75);
            sleep(500);
            turretServo.setPosition(0.825);
            sleep(250);
            clawServo.setPosition(0.4);
            sleep(750);
            leftFourBar.setPosition(0.87);
            rightFourBar.setPosition(0.87);
            sleep(50);
            clawServo.setPosition(0.62);
            sleep(500);
            leftFourBar.setPosition(0.75);
            rightFourBar.setPosition(0.75);
            sleep(500);
            turretServo.setPosition(0.15);
            sleep(500);
            leftFourBar.setPosition(0.35);
            rightFourBar.setPosition(0.35);
            sleep(500);
            clawServo.setPosition(0.4);
            sleep(500);







            break;

        }
    }
}
