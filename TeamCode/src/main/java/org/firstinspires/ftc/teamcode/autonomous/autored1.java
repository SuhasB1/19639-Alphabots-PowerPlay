package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
public class autored1 extends LinearOpMode {

    public static double DISTANCE = 50;
    DcMotor vertMotor;
    DcMotor rhArmMotor;
    DcMotor lhArmMotor;
    Servo fourBarServo;
    Servo fourBarServo2;
    public static double pos = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(35, -60, Math.toRadians(90));

        Pose2d Pose2 = new Pose2d(35, -60,Math.toRadians(-10));

        Pose2d Pose3 = new Pose2d(37, -15, Math.toRadians(90));

        drive.setPoseEstimate(startPose);


        // second test of path
        Trajectory traj3 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(35, -35))
                .build();


        Trajectory traj4 = drive.trajectoryBuilder(traj3.end().plus(new Pose2d(0, 0, Math.toRadians(-90))))
                .strafeTo(new Vector2d(33,2))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end().plus(new Pose2d(0, 0, Math.toRadians(-17))))
                .back(4)
                .build();
        vertMotor = hardwareMap.get(DcMotorEx.class,"vertMotor");
        rhArmMotor = hardwareMap.get(DcMotorEx.class,"rhArmMotor");
        lhArmMotor = hardwareMap.get(DcMotorEx.class, "lhArmMotor");
        vertMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        vertMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rhArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lhArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rhArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lhArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rhArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lhArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rhArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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



        //end of second test of path
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectory(traj3);
            drive.turn(Math.toRadians(0));
            sleep(500);
            drive.followTrajectory(traj4);
            drive.turn(Math.toRadians(-17));
            drive.followTrajectory(traj5);
            updateLiftDriveHor(lhArmMotor,rhArmMotor,600);
            sleep(5000);
            updateLiftDriveVert(2300, vertMotor);
            sleep(5000);

            break;


        }
    }
    public void updateLiftDriveHor(DcMotor motor1, DcMotor motor2, double targetPosition){
        double currentLiftPosition = motor1.getCurrentPosition();  // Get the position of the lift in ticks

        double liftError = targetPosition - currentLiftPosition;  // determine the error in ticks between our current lift position and where it should be
        double liftPower = 0;
        double liftError2 = targetPosition - motor2.getCurrentPosition();
        double liftPower2 = 0;

        if (liftError > 5 || liftError < -5){// Hysteresis band to provide damping/braking zone to reduce oscillations

            liftPower = ((liftError * 0.003) + .05); // PID with P and D factor plus feed forward

            if (liftPower > .5){
                liftPower = .5;
            }
            if (liftPower < -.5){
                liftPower = -.5;
            }
        }
        else {

            liftPower = 0;

        } // end of if-else
        telemetry.addData("power1", liftPower);
        telemetry.update();

        if (liftError2 > 5 || liftError2 < -5){              // Hysteresis band to provide damping/braking zone to reduce oscillations

            liftPower2 = ((liftError2 * 0.003) + .05);   		// PID with P only factor plus feed forward

            if (liftPower2 > .5){
                liftPower2 = .5;
            }
            if (liftPower2 < -.5){
                liftPower2 = -.5;
            }
        }
        else {
            liftPower2 = 0;

        }
        telemetry.addData("power2", liftPower2);
        telemetry.update();

        motor1.setPower(liftPower);
        motor2.setPower(liftPower);


    }

    public void updateLiftDriveVert(double targetPos, DcMotor motor1){
        double targetPosition = targetPos;
        double currentLiftPosition = motor1.getCurrentPosition();  // Get the position of the lift in ticks

        double liftError = targetPosition - currentLiftPosition;  // determine the error in ticks between our current lift position and where it should be
        double liftPower = 0;

        if (liftError > 5 || liftError < -5){// Hysteresis band to provide damping/braking zone to reduce oscillations

            liftPower = ((liftError * 0.1)+ + .1); // PID with P and D factor plus feed forward

            if (liftPower > .9){
                liftPower = .9;
            }
            if (liftPower < -.9){
                liftPower = -.9;
            }
        }
        else {

            liftPower = 0;

        } // end of if-else
        telemetry.addData("power1", liftPower);
        telemetry.update();


        motor1.setPower(liftPower);


    }
}