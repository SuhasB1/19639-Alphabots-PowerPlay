package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.teleopstrafer;

@Config
@Autonomous(name="autoConesTesting")
public class FSMTestTrial_1 extends LinearOpMode {

    ServoImplEx leftFourBar;
    ServoImplEx rightFourBar;
    Servo clawServo;
    Servo turretServo;
    DcMotor vertMotor;
    DcMotorEx lhArmMotor;
    DcMotorEx rhArmMotor;
    int count = 0;
    PIDCoefficients vertCoeffs = new PIDCoefficients(0.016, 0, 0.000375);
    PIDCoefficients horCoeffs = new PIDCoefficients(0.00275, 0, 0.000024);
    public static double vertTargetPos = 0;
    public static double horTargetPos = 0;
    public static double turretPos = 0.15;
    public static double vertDeadband = 20;
    public static double backPos = 0.345;
    public static double downPos = 0.87;
    public static double coneInterval = 0;
    public final double kG = 0.05;
    public static double wait = 1000;
    public double coneCount = 0;


    /*
    1. 4 bar goes to 0.7 while Claw close
2. Hor slides extend
3. While #2 Turn turret while moving 4 bar to 0.87
4. Once hor slides finish extending claw close
5. Hor slides retract
6. While retracting, move 4 bar 0.7 turn turret
7. Once hor slides retract, move 4 bar to 0.34, open claw, move 4 bar back to 0.5
8. Extend vertical
9. While vert extend do the four bar move turret move so it is ready for second cycle.
10. Once vert finishes extending, wait 0.5 secs and then retract.
11. Repeat/exit

     */
    public enum readyPos {
        fourBarClawClose,
        turretFront,
        fourBarClawOpen
    }
    public enum pickUp {
        horExtend,
        fourBarIntake,
        clawClose,
        horRetract,
        fourBarTurretBack,
        fourBarOuttake,
        clawOpen,
        fourBarReset
    }
    public enum pickUpStack {
        horExtend,
        fourBarIntake,
        clawClose,
        horRetract,
        fourBarTurretBack,
        fourBarOuttake,
        clawOpen,
        fourBarReset
    }
    public enum Outtake {
        vertExtend,
        readyPosFunc,
        vertRetract,
        vertRetractP2
    }

    public enum fourBarCycle {
        cycleStart,
        fourBarSet,
        turretFrontClawClose,
        fourBarClawIntake,
        fourBarReset,
        turretBack,
        fourBarBack,
        clawOpen,
        //4barback
    }

    fourBarCycle fourBarCycleState = fourBarCycle.cycleStart;
    readyPos readyPosState = readyPos.fourBarClawClose;
    pickUp pickUpState = pickUp.horExtend;
    pickUpStack pickUpStackState = pickUpStack.horExtend;
    Outtake outtakeState = Outtake.vertExtend;

    ElapsedTime timer1= new ElapsedTime();
    ElapsedTime testTimer= new ElapsedTime();
    ElapsedTime vertTimer= new ElapsedTime();
    int test = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        /*
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
                .lineToSplineHeading(new Pose2d(30, 0.75, Math.toRadians(-13))) //-14.0362435
                .build();

        Trajectory forward = drive.trajectoryBuilder(parshPath2.end())
                .forward(3)
                .build();

        Trajectory backward = drive.trajectoryBuilder(forward.end())
                .back(3)
                .build();


        Trajectory lineForward = drive.trajectoryBuilder(parshPath2.end())
                .lineToConstantHeading(new Vector2d(34.236, -3.25))
                .build();

        Trajectory lineBackward = drive.trajectoryBuilder(lineForward.end())
                .lineToConstantHeading(new Vector2d(30  , 0.75))
                .build();

         */

        leftFourBar = hardwareMap.get(ServoImplEx.class, "leftFourBar");
        rightFourBar = hardwareMap.get(ServoImplEx.class, "rightFourBar");
        clawServo = hardwareMap.get(ServoImplEx.class, "clawServo");
        turretServo = hardwareMap.get(ServoImplEx.class, "turretServo");


        leftFourBar.setDirection(Servo.Direction.REVERSE);

        leftFourBar.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightFourBar.setPwmRange(new PwmControl.PwmRange(500, 2500));

        vertMotor = hardwareMap.get(DcMotorEx.class, "vertMotor");
        vertMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        vertMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rhArmMotor = hardwareMap.get(DcMotorEx.class, "rhArmMotor");
        lhArmMotor = hardwareMap.get(DcMotorEx.class, "lhArmMotor");
        rhArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lhArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rhArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lhArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rhArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lhArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rhArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFController vertController = new PIDFController(vertCoeffs);
        PIDFController horController = new PIDFController(horCoeffs);

        vertController.setTargetPosition(vertTargetPos);
        horController.setTargetPosition(horTargetPos);

        timer1.reset();
        timer1.startTime();
        vertTimer.reset();
        vertTimer.startTime();
        testTimer.reset();
        testTimer.startTime();
        count = 0;
        test = 0;
        coneCount = 0;



        /*
        Claw facing backwards position: 0.15
        Claw facing outwards position: 0.825

        Claw open: 0.4
        Claw close: 0.62
         */


        waitForStart();

        while (opModeIsActive()) {
            vertController.setTargetPosition(vertTargetPos);
            horController.setTargetPosition(horTargetPos);
            vertSlidesPID(vertController);
            horSlidesPID(horController);
            pickUpCycle();
            telemetry.update();
            if(count == 1){
                readyPosCycle();
            }



        }
    }
    public void vertSlidesPID(PIDFController controller){
        double correction = controller.update(vertMotor.getCurrentPosition());
        vertMotor.setPower(correction + kG);
    }
    public void horSlidesPID(PIDFController controller)
    {
        double correction = controller.update((lhArmMotor.getCurrentPosition() + rhArmMotor.getCurrentPosition())/2);
        lhArmMotor.setPower(correction);
        rhArmMotor.setPower(correction);
    }
    public void readyPosCycle() {
        switch (readyPosState) {
            case fourBarClawClose:
                leftFourBar.setPosition(0.75);
                rightFourBar.setPosition(0.75);
                clawServo.setPosition(0.6);
                timer1.reset();
                telemetry.addData("1", testTimer.milliseconds());
                readyPosState = readyPos.turretFront;
                break;
            case turretFront:
                if(timer1.milliseconds() >= 500) {
                    turretServo.setPosition(0.9);
                    timer1.reset();
                    telemetry.addData("2", testTimer.milliseconds());
                    readyPosState = readyPos.fourBarClawOpen;
                }
                break;

            case fourBarClawOpen:
                if (timer1.milliseconds() >= 400) {
                    leftFourBar.setPosition(0.5);
                    rightFourBar.setPosition(0.5);
                    clawServo.setPosition(0.4);

                }
                break;



        }
    }

    public void pickUpCycle(){
        switch (pickUpState) {
            case horExtend:
                leftFourBar.setPosition(downPos);
                rightFourBar.setPosition(downPos);
                sleep(200);
                clawServo.setPosition(0.4);
                sleep(400);
                horTargetPos = 500;
                pickUpState = pickUp.fourBarIntake;
                break;
            case fourBarIntake:
                pickUpState = pickUp.clawClose;
                break;
            case clawClose:
                telemetry.addData("vjgghghjghfjhfh", lhArmMotor.getCurrentPosition());
                if (Math.abs(lhArmMotor.getCurrentPosition() - 500) < 20) {
                    sleep(500);
                    sleep(2000);
                    clawServo.setPosition(0.6);
                    sleep(100);
                    pickUpState = pickUp.horRetract;
                }
                break;
            case horRetract:
                horTargetPos = -20;
                pickUpState = pickUp.fourBarTurretBack;
                break;
            case fourBarTurretBack:
                leftFourBar.setPosition(0.7);
                rightFourBar.setPosition(0.7);
                turretServo.setPosition(turretPos);
                testTimer.reset();
                pickUpState = pickUp.fourBarOuttake;
                break;
              case fourBarOuttake:
                  if (Math.abs(lhArmMotor.getCurrentPosition() - 0) < 30) {
                        telemetry.addData("I Got here3", lhArmMotor.getCurrentPosition());
                        telemetry.update();
                        sleep(200);
                        leftFourBar.setPosition(backPos);
                        rightFourBar.setPosition(backPos);
                        telemetry.addData("I Got here4", lhArmMotor.getCurrentPosition());
                        telemetry.update();
                        pickUpState = pickUp.clawOpen;
                    }
                break;
            case clawOpen:
                sleep(1000);
                clawServo.setPosition(0.4);
                pickUpState = pickUp.fourBarReset;
                break;
            case fourBarReset:
                sleep(200);
                leftFourBar.setPosition(0.5);
                rightFourBar.setPosition(0.5);
                break;

        }
    }
    public void outtakeCycle(){
        switch(outtakeState){
            case vertExtend:
                vertTargetPos = 2290;
                outtakeState = Outtake.readyPosFunc;
                break;
            case readyPosFunc:
                count =1;
                outtakeState = Outtake.vertRetract;
                break;
            case vertRetract:
                if (Math.abs(vertMotor.getCurrentPosition() - 2290) < vertDeadband) {
                    telemetry.addData("ticks", vertMotor.getCurrentPosition());
                    vertTimer.reset();
                    telemetry.addData("time", vertTimer.milliseconds());
                    outtakeState = Outtake.vertRetractP2;
                }
                break;
            case vertRetractP2:
                if (vertTimer.milliseconds() >= wait) {
                    vertTargetPos = 0;
                }
                break;


        }
    }

}

