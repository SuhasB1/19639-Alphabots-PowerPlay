package org.firstinspires.ftc.teamcode.autonomous.fourBar;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Autonomous(name="axon", group="Training")
public class axon extends LinearOpMode {

    ServoImplEx leftFourBar;
    ServoImplEx rightFourBar;
    ServoImplEx turretServo;

    @Override
    public void runOpMode() {

        leftFourBar = hardwareMap.get(ServoImplEx.class,"leftFourBar");
        rightFourBar = hardwareMap.get(ServoImplEx.class,"rightFourBar");
        turretServo = hardwareMap.get(ServoImplEx.class,"turretServo");


        leftFourBar.setDirection(Servo.Direction.REVERSE);

        leftFourBar.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightFourBar.setPwmRange(new PwmControl.PwmRange(500, 2500));


        waitForStart();
        while(opModeIsActive())
        {
            turretServo.setPosition(0.25);

        }
    }
}
