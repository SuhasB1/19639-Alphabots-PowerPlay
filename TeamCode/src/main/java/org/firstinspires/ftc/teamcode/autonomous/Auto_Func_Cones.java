package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
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
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.android.dx.cf.direct.CodeObserver;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="aaWORKINGAuto")
public class Auto_Func_Cones extends LinearOpMode {

    ServoImplEx leftFourBar;
    ServoImplEx rightFourBar;
    Servo clawServo;
    Servo turretServo;
    DcMotor vertMotor;
    DcMotorEx lhArmMotor;
    DcMotorEx rhArmMotor;
    int count = 0;
    int count2 = 0;
    int count3 = 0;
    PIDCoefficients vertCoeffs = new PIDCoefficients(0.016, 0, 0.000375);
    PIDCoefficients horCoeffs = new PIDCoefficients(0.00275, 0, 0.000024);
    double vertTargetPos = 0;
    double horTargetPos = 0;
    public static double backPos = 0.38;
    public static double downPos = 0.88;
    public final double kG = 0.05;
    public double CHANGEABLEhorTargetPos = 640;
    public double fourBarIntakePos = 0.75;


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
    public enum pickUpStack {
        clawClose,
        fourBarUp,
        horRetract,
        fourBarBack,
        turretBack,
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
    public enum AutoFSM {
        mainPath1,
        mainPath2,
        vert1,
        forward1,
        hor1,
        backward1,
        vert2,
        forward2,
        hor2,
        backward2,
        vert3,
        forward3,
        hor3,
        backward3,
        vert4,
        forward4,
        hor4,
        backward4,
        vert5,
    }

    readyPos readyPosState = readyPos.fourBarClawClose;
    AutoFSM autoFSMState = AutoFSM.mainPath1;
    pickUpStack pickUpStackState = pickUpStack.clawClose;
    Outtake outtakeState = Outtake.vertExtend;

    ElapsedTime timer1= new ElapsedTime();
    ElapsedTime testTimer= new ElapsedTime();
    ElapsedTime vertTimer= new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


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

        Pose2d startPose = new Pose2d(35, -60, Math.toRadians(270));



        drive.setPoseEstimate(startPose);


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

        PIDFController vertController = new PIDFController(vertCoeffs);
        PIDFController horController = new PIDFController(horCoeffs);

        vertTargetPos = 0;
        horTargetPos = 0;

        timer1.reset();
        timer1.startTime();
        vertTimer.reset();
        vertTimer.startTime();
        testTimer.reset();
        testTimer.startTime();
        count = 0;
        count2 = 0;
        count3 = 0;
        CHANGEABLEhorTargetPos = 640;
        fourBarIntakePos = 0.75;
        leftFourBar.setPosition(0.5);
        rightFourBar.setPosition(0.5);


        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.left_bumper) {
                drive.followTrajectory(parshPath);
            }
            if(gamepad1.right_bumper) {
                drive.followTrajectory(parshPath2);
            }
            if(gamepad1.dpad_up){
                drive.followTrajectory(forward);
            }
            if(gamepad1.dpad_down){
                drive.followTrajectory(backward);
            }
            vertController.setTargetPosition(vertTargetPos);
            horController.setTargetPosition(horTargetPos);
            vertSlidesPID(vertController);
            horSlidesPID(horController);
            telemetry.update();
            if(count == 1){
                readyPosCycle();
            }
            if(count2 == 1){
                outtakeCycle();
            }
            if(count3 == 1){
                pickUpStackCycle();
            }
            if(gamepad1.triangle){
                outtakeState = Outtake.vertExtend;
                readyPosState = readyPos.fourBarClawClose;
                count2 = 1;
            }
            if(gamepad1.square){
                pickUpStackState = pickUpStack.clawClose;
                count3 = 1;

            }
            switch (autoFSMState) {
                case mainPath1:
                    telemetry.addData("reached", count);
                    drive.followTrajectory(parshPath);
                    autoFSMState = AutoFSM.mainPath2;
                    break;
                case mainPath2:
                    if(!drive.isBusy()) {
                        drive.followTrajectory(parshPath2);
                        autoFSMState = AutoFSM.vert1;
                    }
                    break;
                case vert1:
                    if(!drive.isBusy()) {
                        outtakeState = Outtake.vertExtend;
                        readyPosState = readyPos.fourBarClawClose;
                        CHANGEABLEhorTargetPos = 640;
                        fourBarIntakePos = 0.75;
                        count2 = 1;
                        autoFSMState = AutoFSM.forward1;
                    }
                    break;
                case forward1:
                    if(count2 == 0 && count == 0) {
                        drive.followTrajectory(forward);
                        autoFSMState = AutoFSM.hor1;
                    }
                    break;
                case hor1:
                    if(!drive.isBusy()) {
                        pickUpStackState = pickUpStack.clawClose;
                        count3 = 1;
                        autoFSMState = AutoFSM.backward1;
                    }
                    break;
                case backward1:
                    if(count3 == 0) {
                        drive.followTrajectory(backward);
                        autoFSMState = AutoFSM.vert2;
                    }
                    break;
                case vert2:
                    if(!drive.isBusy()) {
                        outtakeState = Outtake.vertExtend;
                        readyPosState = readyPos.fourBarClawClose;
                        CHANGEABLEhorTargetPos = 640;
                        fourBarIntakePos = 0.775;
                        count2 = 1;
                        autoFSMState = AutoFSM.forward2;
                    }
                    break;
                case forward2:
                    if(count2 == 0 && count == 0) {
                        drive.followTrajectory(forward2);
                        autoFSMState = AutoFSM.hor2;
                    }
                    break;
                case hor2:
                    if(!drive.isBusy()) {
                        pickUpStackState = pickUpStack.clawClose;
                        count3 = 1;
                        autoFSMState = AutoFSM.backward2;
                    }
                    break;
                case backward2:
                    if(count3 == 0) {
                        drive.followTrajectory(backward2);
                        autoFSMState = AutoFSM.vert3;
                    }
                    break;
                case vert3:
                    if(!drive.isBusy()) {
                        outtakeState = Outtake.vertExtend;
                        readyPosState = readyPos.fourBarClawClose;
                        CHANGEABLEhorTargetPos = 630;
                        fourBarIntakePos = 0.81;
                        count2 = 1;
                        autoFSMState = AutoFSM.forward3;
                    }
                    break;
                case forward3:
                    if(count2 == 0 && count == 0) {
                        drive.followTrajectory(forward3);
                        autoFSMState = AutoFSM.hor3;
                    }
                    break;
                case hor3:
                    if(!drive.isBusy()) {
                        pickUpStackState = pickUpStack.clawClose;
                        count3 = 1;
                        autoFSMState = AutoFSM.backward3;
                    }
                    break;
                case backward3:
                    if(count3 == 0) {
                        drive.followTrajectory(backward3);
                        autoFSMState = AutoFSM.vert4;
                    }
                    break;
                case vert4:
                    if(!drive.isBusy()) {
                        outtakeState = Outtake.vertExtend;
                        readyPosState = readyPos.fourBarClawClose;
                        CHANGEABLEhorTargetPos = 0;
                        fourBarIntakePos = 0.5;
                        count2 = 1;
                    }
                    break;

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
                leftFourBar.setPosition(fourBarIntakePos);
                rightFourBar.setPosition(fourBarIntakePos);
                clawServo.setPosition(0.62);
                timer1.reset();
                telemetry.addData("1", testTimer.milliseconds());
                readyPosState = readyPos.turretFront;
                break;
            case turretFront:
                if(timer1.milliseconds() >= 500) {
                    turretServo.setPosition(0.8);
                    timer1.reset();
                    telemetry.addData("2", testTimer.milliseconds());
                    readyPosState = readyPos.fourBarClawOpen;
                }
                break;
            case fourBarClawOpen:
                if (timer1.milliseconds() >= 400) {
                    clawServo.setPosition(0.4);
                    horTargetPos = CHANGEABLEhorTargetPos;
                    if (Math.abs(lhArmMotor.getCurrentPosition() - CHANGEABLEhorTargetPos) < 20) {
                        count = 0;
                    }
                }
                break;



        }
    }
    public void outtakeCycle(){
        switch(outtakeState){
            case vertExtend:
                count =1;
                outtakeState = Outtake.readyPosFunc;
                break;
            case readyPosFunc:
                vertTargetPos = 2290;
                outtakeState = Outtake.vertRetract;
                break;
            case vertRetract:
                if (Math.abs(vertMotor.getCurrentPosition() - 2290) < 20) {
                    telemetry.addData("ticks", vertMotor.getCurrentPosition());
                    vertTimer.reset();
                    telemetry.addData("time", vertTimer.milliseconds());
                    outtakeState = Outtake.vertRetractP2;
                }
                break;
            case vertRetractP2:
                if (vertTimer.milliseconds() >= 200) {
                    vertTargetPos = -25;
                    if (Math.abs(lhArmMotor.getCurrentPosition() - 0) < 30) {
                        count2 = 0;
                    }
                }
                break;


        }
    }
    public void pickUpStackCycle(){
        switch (pickUpStackState) {
            case clawClose:
                clawServo.setPosition(0.6);
                    sleep(100);
                    pickUpStackState = pickUpStack.fourBarUp;
                break;
            case fourBarUp:
                pickUpStackState = pickUpStack.horRetract;
                break;
            case horRetract:
                leftFourBar.setPosition(0.6);
                rightFourBar.setPosition(0.6);
                sleep(400);
                horTargetPos = 200;
                pickUpStackState = pickUpStack.fourBarBack;
                break;
            case fourBarBack:
                telemetry.addData("I Got here", lhArmMotor.getCurrentPosition());
                if (Math.abs(lhArmMotor.getCurrentPosition() - 200) < 45) {
                    telemetry.addData("I Got here2", lhArmMotor.getCurrentPosition());
                    leftFourBar.setPosition(0.75);
                    rightFourBar.setPosition(0.75);
                    sleep(200);
                    turretServo.setPosition(0.1);
                    horTargetPos = -40;
                    pickUpStackState = pickUpStack.turretBack;
                }
                break;
            case turretBack:
                pickUpStackState = pickUpStack.fourBarOuttake;
                break;
            case fourBarOuttake:
                telemetry.addData("I Got here", lhArmMotor.getCurrentPosition());
                if (Math.abs(lhArmMotor.getCurrentPosition() - 0) < 45) {
                    telemetry.addData("I Got here", lhArmMotor.getCurrentPosition());
                    sleep(200);
                    leftFourBar.setPosition(backPos);
                    rightFourBar.setPosition(backPos);
                    pickUpStackState = pickUpStack.clawOpen;
                }
                break;
            case clawOpen:
                sleep(600);
                clawServo.setPosition(0.4);
                pickUpStackState = pickUpStack.fourBarReset;
                break;
            case fourBarReset:
                sleep(300);
                leftFourBar.setPosition(0.5);
                rightFourBar.setPosition(0.5);
                count3 = 0;
                break;


        }
    }
}

