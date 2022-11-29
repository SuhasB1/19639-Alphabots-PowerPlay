package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config
@Autonomous(name="autored1",group = "drive")
public class autored1 extends LinearOpMode {

    public static double DISTANCE = 50;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(35, -60, Math.toRadians(90));

        Pose2d Pose2 = new Pose2d(35, -60,Math.toRadians(-10));

        Pose2d Pose3 = new Pose2d(37, -15, Math.toRadians(90));

        drive.setPoseEstimate(startPose);
        //first test of path
        Trajectory traj1 = drive.trajectoryBuilder(startPose) //splines may have to be in separate trajectories test it out!
                .lineToLinearHeading(new Pose2d(34,-12, Math.toRadians(-10)))

                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeTo(new Vector2d(30, 2))
                .build();
        // end of first test of path

        // second test of path
        Trajectory traj3 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(35, -38))
                .build();


        Trajectory traj4 = drive.trajectoryBuilder(traj3.end().plus(new Pose2d(0, 0, Math.toRadians(-90))))
                .strafeTo(new Vector2d(33,3))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end().plus(new Pose2d(0, 0, Math.toRadians(-13))))
                .back(4)
                .build();

        //end of second test of path
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectory(traj3);
            drive.turn(Math.toRadians(0));
            sleep(500);
            drive.followTrajectory(traj4);
            drive.turn(Math.toRadians(-13));
            drive.followTrajectory(traj5);

            break;


        }
    }
}