package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.android.dx.cf.direct.CodeObserver;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Vector;

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
    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;
    double parkingPos;
//    DcMotor vertMotor;
//    DcMotor rhArmMotor;
//    DcMotor lhArmMotor;
//    Servo fourBarServo;
//    Servo fourBarServo2;
//    CRServo intakeServo1;
//    CRServo intakeServo2;
    DcMotorEx motor1;


    public static double x = 30;
    public static double y = 0.75;

    public static double pos = 0.77;
    public static double angle = -13;
    double maxVertPower = 0.7;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    SleeveDetection.ParkingPosition snapshot = SleeveDetection.ParkingPosition.CENTER; //default
    public static double kD = 0.000175;
    public static double kP = 0.017;
    public static double kG = 0.05;
    final public double kI = 0;
    public static double targetPos = 2000; //2290 (high)
    public static double maxMotorTicks = 2000;
    double derivative;
    double lastError;
    double integralSum = 0;
    public ElapsedTime runtime = new ElapsedTime();
    PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);







    @Override
    public void runOpMode() throws InterruptedException {

        motor1 = hardwareMap.get(DcMotorEx.class,"vertMotor");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PIDFController controller = new PIDFController(coeffs);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(35, -60, Math.toRadians(270));



        drive.setPoseEstimate(startPose);


        // second test of path
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(35, -2))
                .build();


        Trajectory traj2 = drive.trajectoryBuilder(traj1.end().plus(new Pose2d(0, 0, Math.toRadians(-105))))
                .strafeLeft(6.75)

                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .back(7)

                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end().plus(new Pose2d(0, 0, Math.toRadians(15))))
                .forward(4)
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .strafeRight(24)
                .build();

        Trajectory splinePath = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(35, 4, Math.toRadians(-16)))
                .build();

        Trajectory back = drive.trajectoryBuilder(splinePath.end())
                .back(4)
                .build();



        Trajectory parshPath = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(35, -20))
                .build();

        Trajectory parshPath2 = drive.trajectoryBuilder(parshPath.end())
                .lineToSplineHeading(new Pose2d(x, y, Math.toRadians(angle))) //-14.0362435
                .build();

        Trajectory forward = drive.trajectoryBuilder(parshPath2.end())
                .forward(3)
                .build();

        Trajectory backward = drive.trajectoryBuilder(forward.end())
                .back(3)
                .build();

        Trajectory forward2 = drive.trajectoryBuilder(backward.end())
                .forward(3)
                .build();

        Trajectory backward2 = drive.trajectoryBuilder(forward2.end())
                .back(3)
                .build();

        Trajectory forward3 = drive.trajectoryBuilder(backward2.end())
                .forward(3)
                .build();

        Trajectory backward3 = drive.trajectoryBuilder(forward3.end())
                .back(3)
                .build();

        Trajectory forward4 = drive.trajectoryBuilder(backward3.end())
                .forward(3)
                .build();

        Trajectory backward4 = drive.trajectoryBuilder(forward4.end())
                .back(3)
                .build();

        Trajectory forward5 = drive.trajectoryBuilder(backward4.end())
                .forward(3)
                .build();

        Trajectory backward5 = drive.trajectoryBuilder(forward5.end())
                .back(3)
                .build();

        Trajectory lineForward = drive.trajectoryBuilder(parshPath2.end())
                .lineToConstantHeading(new Vector2d(34.236, -3.25))
                .build();

        Trajectory lineBackward = drive.trajectoryBuilder(lineForward.end())
                .lineToConstantHeading(new Vector2d(30  , 0.75))
                .build();



//
//        vertMotor = hardwareMap.get(DcMotorEx.class,"vertMotor");
//        rhArmMotor = hardwareMap.get(DcMotorEx.class,"rhArmMotor");
//        lhArmMotor = hardwareMap.get(DcMotorEx.class, "lhArmMotor");
//        vertMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        vertMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        vertMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        vertMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rhArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lhArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rhArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lhArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rhArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        lhArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rhArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        fourBarServo = hardwareMap.servo.get("fourBarServo");
//        fourBarServo2 = hardwareMap.servo.get("fourBarServo2");
//        fourBarServo.setDirection(Servo.Direction.REVERSE);
//        vertSlidesPID(0);
//        horSlidesPID(0);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        camera.setPipeline(sleeveDetection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.SIDEWAYS_RIGHT); //upright before
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);



//        fourBarServo.setPosition(pos * 0.25);
//        fourBarServo2.setPosition(pos * 0.25);
//        fourBarServo.setPosition(pos * 0.5);
//        fourBarServo2.setPosition(pos * 0.5);
//        fourBarServo.setPosition(pos * 0.75);
//        fourBarServo2.setPosition(pos * 0.75);
//        fourBarServo.setPosition(pos);
//        fourBarServo2.setPosition(pos);
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime Analysis", sleeveDetection.getPosition());
            telemetry.update();
            sleep(150);
            /*
            Trajectory building in init
             */



        }
        snapshot = sleeveDetection.getPosition();

        telemetry.addData("Snapshot POST-START Analysis", snapshot);
        telemetry.update();





        waitForStart();
        TelemetryPacket packet = new TelemetryPacket();
        dashboard.setTelemetryTransmissionInterval(25);
        packet.put("position", motor1.getCurrentPosition());
        packet.put("targetPosition", targetPos);
        dashboard.sendTelemetryPacket(packet);
        controller.setTargetPosition(targetPos);
        while(opModeIsActive()){
            switch(snapshot){
                case LEFT:
//                    drive.followTrajectory(traj1);
//                    sleep(500);
//                    drive.turn(Math.toRadians(-105));
//                    drive.followTrajectory(traj2);
//                    drive.followTrajectory(traj3);
//                    drive.turn(Math.toRadians(15));
//                    drive.followTrajectory(traj4);
//                    drive.followTrajectory(traj5);
//                    drive.followTrajectory(splinePath);
//                    drive.followTrajectory(back);
                    drive.followTrajectory(parshPath);
                    drive.followTrajectory(parshPath2);
                    drive.followTrajectory(forward);
//                    sleep(5000);
//                    drive.followTrajectory(lineForward);
//                    sleep(2000);
//                    drive.followTrajectory(lineBackward);
//                    sleep(2000);
//                    drive.followTrajectory(lineForward);
//                    sleep(2000);
//                    drive.followTrajectory(lineBackward);
//                    sleep(2000);
//                    drive.followTrajectory(lineForward);
//                    sleep(2000);
//                    drive.followTrajectory(lineBackward);
//                    sleep(2000);
//                    drive.followTrajectory(lineForward);
//                    sleep(2000);
//                    drive.followTrajectory(lineBackward);
//                    sleep(2000);
//                    drive.followTrajectory(lineForward);
//                    sleep(2000);
//                    drive.followTrajectory(lineBackward);
//                    drive.followTrajectory(forward);
//                    sleep(5000);
//                    drive.followTrajectory(backward);
//                    sleep(5000);
//                    drive.followTrajectory(forward2);
//                    sleep(5000);
//                    drive.followTrajectory(backward2);
//                    sleep(5000);
//                    drive.followTrajectory(forward3);
//                    sleep(5000);
//                    drive.followTrajectory(backward3);
//                    sleep(5000);
//                    drive.followTrajectory(forward4);
//                    sleep(5000);
//                    drive.followTrajectory(backward4);
//                    sleep(5000);
//                    drive.followTrajectory(forward5);
//                    sleep(5000);
//                    drive.followTrajectory(backward5);

                    sleep(2000);

                    break;
                case RIGHT:
                    drive.followTrajectory(parshPath);
                    drive.followTrajectory(parshPath2);
                    sleep(5000);
                    drive.followTrajectory(lineForward);
                    sleep(2000);
                    drive.followTrajectory(lineBackward);
                    sleep(2000);
                    drive.followTrajectory(lineForward);
                    sleep(2000);
                    drive.followTrajectory(lineBackward);
                    sleep(2000);
                    drive.followTrajectory(lineForward);
                    sleep(2000);
                    drive.followTrajectory(lineBackward);
                    sleep(2000);
                    drive.followTrajectory(lineForward);
                    sleep(2000);
                    drive.followTrajectory(lineBackward);
                    sleep(2000);
                    drive.followTrajectory(lineForward);
                    sleep(2000);
                    drive.followTrajectory(lineBackward);
//                    drive.followTrajectory(forward);
//                    sleep(5000);
//                    drive.followTrajectory(backward);
//                    sleep(5000);
//                    drive.followTrajectory(forward2);
//                    sleep(5000);
//                    drive.followTrajectory(backward2);
//                    sleep(5000);
//                    drive.followTrajectory(forward3);
//                    sleep(5000);
//                    drive.followTrajectory(backward3);
//                    sleep(5000);
//                    drive.followTrajectory(forward4);
//                    sleep(5000);
//                    drive.followTrajectory(backward4);
//                    sleep(5000);
//                    drive.followTrajectory(forward5);
//                    sleep(5000);
//                    drive.followTrajectory(backward5);




                    break;
                case CENTER:

                    break;

            }
            break;

        }


        }
    public void vertSlidesPID(PIDFController controller){
        double correction = controller.update(motor1.getCurrentPosition());
        motor1.setPower(correction + kG);

    }

    public void vertSlides(double targetPos){
        runtime.startTime();
        double currentLiftPosition = motor1.getCurrentPosition();  // Get the position of the lift in ticks

        double liftError = targetPos - currentLiftPosition;  // determine the error in ticks between our current lift position and where it should be
        double liftPower = 0;
        derivative = (liftError - lastError) / runtime.seconds();
        lastError = liftError;

        runtime.reset();



        if (liftError > 10 || liftError < -10){// Hysteresis band to provide damping/braking zone to reduce oscillations

            liftPower = ((liftError * kP)  + (derivative * kD) +  kG); // PID with P and D factor plus feed forward

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


        motor1.setPower(liftPower);


    }
}