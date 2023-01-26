package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.android.dx.cf.direct.CodeObserver;
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
@Autonomous
public class autored1 extends LinearOpMode {

    public static double DISTANCE = 50;
    DcMotor vertMotor;
    DcMotor rhArmMotor;
    DcMotor lhArmMotor;
    Servo fourBarServo;
    Servo fourBarServo2;
    CRServo intakeServo1;
    CRServo intakeServo2;

    public static double kD = 0;
    public static double pos = 0.77;
    double maxVertPower = 0.7;
    public static double targetPos = 0;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();




    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(35, -60, Math.toRadians(90));



        drive.setPoseEstimate(startPose);


        // second test of path
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(35, -2))
                .build();


        Trajectory traj2 = drive.trajectoryBuilder(traj1.end().plus(new Pose2d(0, 0, Math.toRadians(-102))))
                .back(4)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeLeft(6)
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
        vertSlidesPID(0);
        horSlidesPID(0);



//        fourBarServo.setPosition(pos * 0.25);
//        fourBarServo2.setPosition(pos * 0.25);
//        fourBarServo.setPosition(pos * 0.5);
//        fourBarServo2.setPosition(pos * 0.5);
//        fourBarServo.setPosition(pos * 0.75);
//        fourBarServo2.setPosition(pos * 0.75);
//        fourBarServo.setPosition(pos);
//        fourBarServo2.setPosition(pos);



        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectory(traj1);
            sleep(500);
            drive.turn(Math.toRadians(-102));
            drive.followTrajectory(traj2);
            drive.followTrajectory(traj3);
            vertSlidesPID(2000);
            sleep(2000);
            vertSlidesPID(0);
            sleep(2000);







            break;


        }
    }
    public void horSlidesPID(double targetPosition){
        double currentLiftPosition = rhArmMotor.getCurrentPosition();  // Get the position of the lift in ticks

        double liftError = (targetPosition-5) - currentLiftPosition;  // determine the error in ticks between our current lift position and where it should be
        double liftPower = 0;
        double liftError2 = (targetPosition-5) - lhArmMotor.getCurrentPosition();
        double liftPower2 = 0;

        if (liftError > 5 || liftError < -5){// Hysteresis band to provide damping/braking zone to reduce oscillations

            liftPower = ((liftError * 0.005)); // PID with P and D factor plus feed forward

            if (liftPower > .65){
                liftPower = .65;
            }
            if (liftPower < -.65){
                liftPower = -.65;
            }
        }
        else {

            liftPower = 0;

        } // end of if-else

        if (liftError2 > 5 || liftError2 < -5){              // Hysteresis band to provide damping/braking zone to reduce oscillations

            liftPower2 = ((liftError2 * 0.005));   		// PID with P only factor plus feed forward

            if (liftPower2 > .65){
                liftPower2 = .65;
            }
            if (liftPower2 < -.65){
                liftPower2 = -.65;
            }
        }
        else {
            liftPower2 = 0;

        }

        rhArmMotor.setPower(liftPower);
        lhArmMotor.setPower(liftPower);


    }

    public void vertSlidesPID(double targetPosition){
        double currentLiftPosition = vertMotor.getCurrentPosition();  // Get the position of the lift in ticks

        double liftError = targetPosition - currentLiftPosition;  // determine the error in ticks between our current lift position and where it should be
        double liftPower = 0;

        if (liftError > 10 || liftError < -10){// Hysteresis band to provide damping/braking zone to reduce oscillations

            liftPower = ((liftError * 0.03) +  .1); // PID with P and D factor plus feed forward

            if (liftPower > maxVertPower){
                liftPower = maxVertPower;
            }
            if (liftPower < -maxVertPower){
                liftPower = -maxVertPower;
            }
        }
        else {

            liftPower = 0;

        } // end of if-else


        vertMotor.setPower(liftPower);


    }

    public void servoFuncLoop(double pos){
        for(double i = pos/2; i <= pos; i += 0.01){
            fourBarServo.setPosition(i);
            fourBarServo2.setPosition(i);
            sleep(10);

        }
    }
    public void servoFunc(double pos){

        fourBarServo.setPosition(pos * 0.7);
        fourBarServo2.setPosition(pos * 0.7);
        sleep(200);
        fourBarServo.setPosition(pos * 0.8);
        fourBarServo2.setPosition(pos * 0.8);
        sleep(200);
        fourBarServo.setPosition(pos * 0.85);
        fourBarServo2.setPosition(pos * 0.85);
        sleep(200);
        fourBarServo.setPosition(pos * 0.875);
        fourBarServo2.setPosition(pos * 0.875);
        sleep(200);
        fourBarServo.setPosition(pos * 0.9);
        fourBarServo2.setPosition(pos * 0.9);
        sleep(200);
        fourBarServo.setPosition(pos * 0.91);
        fourBarServo2.setPosition(pos * 0.91);
        sleep(200);
        fourBarServo.setPosition(pos * 0.92);
        fourBarServo2.setPosition(pos * 0.92);
        sleep(200);
        fourBarServo.setPosition(pos * 0.93);
        fourBarServo2.setPosition(pos * 0.93);
        sleep(200);
        fourBarServo.setPosition(pos * 0.94);
        fourBarServo2.setPosition(pos * 0.94);
        sleep(200);
        fourBarServo.setPosition(pos * 0.95);
        fourBarServo2.setPosition(pos * 0.95);
        sleep(200);
        fourBarServo.setPosition(pos);
        fourBarServo2.setPosition(pos);
    }
    public void servoOuttakeFunc(double pos2){
        intakeServo1.setPower(0.5);
        intakeServo2.setPower(0.5);
        fourBarServo.setPosition(pos2);
        fourBarServo2.setPosition(pos2);
        intakeServo1.setPower(-0.2);
        intakeServo2.setPower(-0.5);
        sleep(500);
        fourBarServo.setPosition(0.5);
        fourBarServo2.setPosition(0.5);
        sleep(100);
        intakeServo1.setPower(0);
        intakeServo2.setPower(0);

    }

}