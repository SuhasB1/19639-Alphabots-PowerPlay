package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="auto1", group="Training")
public class auto extends LinearOpMode {


    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
                .forward(50)
                .build();


        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectory(trajectoryForward);
        }


    }
}
