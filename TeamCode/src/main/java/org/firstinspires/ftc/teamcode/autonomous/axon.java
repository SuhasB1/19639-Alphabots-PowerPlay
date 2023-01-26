package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Autonomous(name="axon", group="Training")
public class axon extends LinearOpMode {

    ServoImplEx axon;

    @Override
    public void runOpMode() {

        axon = hardwareMap.get(ServoImplEx.class, "axonservo");
        axon.setPwmRange(new PwmControl.PwmRange(500, 2500));


        waitForStart();
        while(opModeIsActive())
        {

            axon.setPosition(0.5);

        }
    }
}
