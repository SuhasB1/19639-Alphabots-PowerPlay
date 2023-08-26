package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Disabled

@Autonomous(name="aaWORKINGAutoMIDDDDDD")
public class Auto_Func_ConesMID extends LinearOpMode {

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
    PIDCoefficients vertCoeffs = new PIDCoefficients(-0.03, 0, -0.00025);
    PIDCoefficients horCoeffs = new PIDCoefficients(0.00305, 0, 0.00001);
    double vertTargetPos = 0;
    double horTargetPos = 0;
    public static double backPos = 0.372;
    public static double downPos = 0.88;
    public final double kG = 0.05;
    public double CHANGEABLEhorTargetPos = 650;
    public double fourBarIntakePos = 0.75;
    public static double angle = 15;
    public static double y = -20;
    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;
    int snapshotPos = 1;
    int timerCount = 0;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    SleeveDetection.ParkingPosition snapshot = SleeveDetection.ParkingPosition.CENTER; //default


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
        mainPath3,
        mainPath4,
        mainPath5,
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
        mainEnd,
        mainDecision,
        mainFinal,
        ending
    }

    readyPos readyPosState = readyPos.fourBarClawClose;
    AutoFSM autoFSMState = AutoFSM.mainPath1;
    pickUpStack pickUpStackState = pickUpStack.clawClose;
    Outtake outtakeState = Outtake.vertExtend;

    ElapsedTime timer1 = new ElapsedTime();
    ElapsedTime testTimer = new ElapsedTime();
    ElapsedTime vertTimer = new ElapsedTime();

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



        Trajectory mainPath = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(35, -12))
                .build();

        Trajectory mainPath2 = drive.trajectoryBuilder(mainPath.end())
                .lineToConstantHeading(new Vector2d(35, -30))
                .build();
        Trajectory mainPath3 = drive.trajectoryBuilder(mainPath2.end())
                .lineToSplineHeading(new Pose2d(28.5, y, Math.toRadians(angle))) //-14.0362435
                .build();
        Trajectory mainPath4 = drive.trajectoryBuilder(mainPath3.end())
                .lineToLinearHeading(new Pose2d(30, -12, Math.toRadians(0)))
                .build();
        Trajectory mainPath5 = drive.trajectoryBuilder(mainPath4.end())
                .back(20)
                .build();
        Trajectory forward = drive.trajectoryBuilder(mainPath3.end())
                .forward(5.2)
                .build();

        Trajectory backward = drive.trajectoryBuilder(forward.end())
                .back(5.2)
                .build();

        Trajectory forward2 = drive.trajectoryBuilder(backward.end())
                .forward(5.2)
                .build();

        Trajectory backward2 = drive.trajectoryBuilder(forward2.end())
                .back(5.2)
                .build();

        Trajectory forward3 = drive.trajectoryBuilder(backward2.end())
                .forward(5.2)
                .build();

        Trajectory backward3 = drive.trajectoryBuilder(forward3.end())
                .back(5.2)
                .build();

        Trajectory forward4 = drive.trajectoryBuilder(backward3.end())
                .forward(5.2)
                .build();

        Trajectory backward4 = drive.trajectoryBuilder(forward4.end())
                .back(5.2)
                .build();

        Trajectory mainend = drive.trajectoryBuilder(backward4.end())
                .lineToLinearHeading(new Pose2d(30, -15, Math.toRadians(0)))
                .build();


        Trajectory rightend = drive.trajectoryBuilder(backward4.end())
                .lineToLinearHeading(new Pose2d(30, -15, Math.toRadians(0)))
                .build();

        Trajectory rightend2 = drive.trajectoryBuilder(rightend.end())
                .forward(25)
                .build();

        Trajectory leftend = drive.trajectoryBuilder(backward4.end())
                .lineToLinearHeading(new Pose2d(30, -15, Math.toRadians(0)))
                .build();

        Trajectory leftend2 = drive.trajectoryBuilder(leftend.end())
                .back(20)
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        camera.setPipeline(sleeveDetection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.SIDEWAYS_RIGHT); //upright before

            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

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
        timerCount = 0;
        CHANGEABLEhorTargetPos = 640;
        fourBarIntakePos = 0.75;
        leftFourBar.setPosition(0.5);
        rightFourBar.setPosition(0.5);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Realtime Analysis", sleeveDetection.getPosition());
            telemetry.update();
            sleep(150);
            /*
            Trajectory building in init
             */
        }
        waitForStart();
        snapshot = sleeveDetection.getPosition();

        telemetry.addData("Snapshot POST-START Analysis", snapshot);
        telemetry.update();
        switch (snapshot) {
            case LEFT:
                snapshotPos = 2;
                break;
            case RIGHT:
                snapshotPos = 3;
                break;
            case CENTER:
                snapshotPos = 1;
                break;
        }
        telemetry.addLine();
        telemetry.addData("snapshotPos: ", snapshotPos);
        telemetry.update();

        camera.stopStreaming();
        while (opModeIsActive() && timerCount == 0) {
            if (gamepad1.left_bumper) {
                drive.followTrajectory(mainPath);
            }
            if (gamepad1.right_bumper) {
                drive.followTrajectory(mainPath2);
            }
            if (gamepad1.dpad_up) {
                drive.followTrajectory(forward);
            }
            if (gamepad1.dpad_down) {
                drive.followTrajectory(backward);
            }
            vertController.setTargetPosition(vertTargetPos);
            horController.setTargetPosition(horTargetPos);
            vertSlidesPID(vertController);
            horSlidesPID(horController);
            if (count == 1) {
                readyPosCycle();
            }
            if (count2 == 1) {
                outtakeCycle();
            }
            if (count3 == 1) {
                pickUpStackCycle();
            }
            if (gamepad1.triangle) {
                outtakeState = Outtake.vertExtend;
                readyPosState = readyPos.fourBarClawClose;
                count2 = 1;
            }
            if (gamepad1.square) {
                pickUpStackState = pickUpStack.clawClose;
                count3 = 1;

            }
            switch (autoFSMState) {
                case mainPath1:
                    drive.followTrajectory(mainPath);
                    telemetry.addLine("Ran mainpath1");
                    telemetry.update();
                    autoFSMState = AutoFSM.mainPath2;
                    break;
                case mainPath2:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(mainPath2);
                        telemetry.addLine("Ran mainpath2");
                        telemetry.update();
                        autoFSMState = AutoFSM.mainPath3;
                    }
                    break;
                case mainPath3:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(mainPath3);
                        telemetry.addLine("Ran mainpath3");
                        telemetry.update();
                        autoFSMState = AutoFSM.mainPath4;
                    }
                    break;
                case mainPath4:
                        autoFSMState = AutoFSM.mainPath5;
                    break;
                case mainPath5:
                        autoFSMState = AutoFSM.vert1;
                    break;
                case vert1:
                    if (!drive.isBusy()) {
                        outtakeState = Outtake.vertExtend;
                        telemetry.addLine("Ran vert1");
                        telemetry.update();
                        readyPosState = readyPos.fourBarClawClose;
                        telemetry.addLine("Ran fourbarclawclose");
                        telemetry.update();
                        CHANGEABLEhorTargetPos = 645;
                        fourBarIntakePos = 0.75;
                        count2 = 1;
                        autoFSMState = AutoFSM.forward1;
                    }
                    break;
                case forward1:
                    if (count == 0 && count2 == 0) {
                        drive.followTrajectory(forward);
                        telemetry.addData("pose", drive.getPoseEstimate());
                        telemetry.addLine();
                        telemetry.addLine("Ran foward");
                        telemetry.update();
                        autoFSMState = AutoFSM.hor1;
                    }
                    break;
                case hor1:
                    if (!drive.isBusy()) {
                        pickUpStackState = pickUpStack.clawClose;
                        telemetry.addLine("Ran hor1");
                        telemetry.update();
                        count3 = 1;
                        autoFSMState = AutoFSM.backward1;
                    }
                    break;
                case backward1:
                    if (count3 == 0) {
                        drive.followTrajectory(backward);
                        telemetry.addLine("Ran backward1");
                        telemetry.update();
                        autoFSMState = AutoFSM.vert2;
                    }
                    break;
                case vert2:
                    if (!drive.isBusy()) {
                        outtakeState = Outtake.vertExtend;
                        telemetry.addLine("Ran vert2");
                        telemetry.update();
                        readyPosState = readyPos.fourBarClawClose;
                        telemetry.addLine("Ran fourbarclawclose");
                        telemetry.update();
                        CHANGEABLEhorTargetPos = 635;
                        fourBarIntakePos = 0.78;
                        count2 = 1;
                        autoFSMState = AutoFSM.forward2;
                    }
                    break;
                case forward2:
                    if (count == 0 && count2 == 0) {
                        telemetry.addLine("Ran forward2");
                        telemetry.update();
                        drive.followTrajectory(forward2);
                        autoFSMState = AutoFSM.hor2;
                    }
                    break;
                case hor2:
                    if (!drive.isBusy()) {
                        telemetry.addLine("Ran hor2");
                        telemetry.update();
                        pickUpStackState = pickUpStack.clawClose;
                        count3 = 1;
                        autoFSMState = AutoFSM.backward2;
                    }
                    break;
                case backward2:
                    if (count3 == 0) {
                        telemetry.addLine("Ran back2");
                        telemetry.update();
                        drive.followTrajectory(backward2);
                        autoFSMState = AutoFSM.vert3;
                    }
                    break;
                case vert3:
                    if (!drive.isBusy()) {
                        telemetry.addLine("Ran vert3");
                        telemetry.update();
                        outtakeState = Outtake.vertExtend;
                        readyPosState = readyPos.fourBarClawClose;
                        CHANGEABLEhorTargetPos = 625;
                        fourBarIntakePos = 0.815;
                        count2 = 1;
                        autoFSMState = AutoFSM.forward3;
                    }
                    break;
                case forward3:
                    if (count == 0 && count2 == 0) {
                        telemetry.addLine("Ran for 3");
                        telemetry.update();
                        drive.followTrajectory(forward3);
                        autoFSMState = AutoFSM.hor3;
                    }
                    break;
                case hor3:
                    if (!drive.isBusy()) {
                        telemetry.addLine("Ran hor3");
                        telemetry.update();
                        pickUpStackState = pickUpStack.clawClose;
                        count3 = 1;
                        autoFSMState = AutoFSM.backward3;
                    }
                    break;
                case backward3:
                    if (count3 == 0) {
                        telemetry.addLine("Ran back2");
                        telemetry.update();
                        drive.followTrajectory(backward3);
                        //autoFSMState = AutoFSM.vert4;
                        //IMPORTANT
                        //IF YOU MAKE 1+4 MAKE SURE TO UNCOMMENT ^ AND COMMENT LINE BELOW
                        autoFSMState = AutoFSM.vert5;
                    }
                    break;
                    /*
                case vert4:
                    if (!drive.isBusy()) {
                        telemetry.addLine("Ran vert4");
                        telemetry.update();
                        outtakeState = Outtake.vertExtend;
                        readyPosState = readyPos.fourBarClawClose;
                        CHANGEABLEhorTargetPos = 620;
                        fourBarIntakePos = 0.84;
                        count2 = 1;
                        autoFSMState = AutoFSM.forward4;
                    }
                    break;
                case forward4:
                    if (count == 0 && count2 == 0) {
                        telemetry.addLine("Ran forward4");
                        telemetry.update();
                        drive.followTrajectory(forward4);
                        autoFSMState = AutoFSM.hor4;
                    }
                    break;
                case hor4:
                    if (!drive.isBusy()) {
                        telemetry.addLine("Ran hor4");
                        telemetry.update();
                        pickUpStackState = pickUpStack.clawClose;
                        count3 = 1;
                        autoFSMState = AutoFSM.backward4;
                    }
                    break;
                case backward4:
                    if (count3 == 0) {
                        telemetry.addLine("Ran back4");
                        telemetry.update();
                        drive.followTrajectory(backward4);
                        autoFSMState = AutoFSM.vert5;
                    }
                    break; */
                case vert5:
                    if (!drive.isBusy()) {
                        outtakeState = Outtake.vertExtend;
                        readyPosState = readyPos.fourBarClawClose;
                        CHANGEABLEhorTargetPos = -15;
                        fourBarIntakePos = 0.5;
                        count2 = 1;
                        telemetry.addLine("vert5");
                        telemetry.update();
                        autoFSMState = AutoFSM.mainEnd;
                    }
                    break;
                case mainEnd:
                    telemetry.addLine("outside main");
                    telemetry.update();
                    if (count == 0 && count2 == 0) {
                        telemetry.addLine("in main");
                        telemetry.update();
                        drive.followTrajectory(mainPath4);
                        autoFSMState = AutoFSM.mainDecision;
                    }
                    break;
                case mainDecision:
                        autoFSMState = AutoFSM.mainFinal;
                    break;
                case mainFinal:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(mainPath5);
                        autoFSMState = AutoFSM.ending;
                    }
                    break;
                case ending:
                    if (!drive.isBusy()) {
                        timerCount = 1; //essentially a break statement
                    }



            }
        }
    }
    public void vertSlidesPID (PIDFController controller){
        double correction = controller.update(vertMotor.getCurrentPosition());
        vertMotor.setPower(correction + kG);
    }
    public void horSlidesPID (PIDFController controller)
    {
        double correction = controller.update((lhArmMotor.getCurrentPosition() + rhArmMotor.getCurrentPosition()) / 2);
        lhArmMotor.setPower(correction);
        rhArmMotor.setPower(correction);
    }
    public void readyPosCycle () {
        switch (readyPosState) {
            case fourBarClawClose:
                leftFourBar.setPosition(fourBarIntakePos);
                rightFourBar.setPosition(fourBarIntakePos);
                clawServo.setPosition(0.7);
                timer1.reset();
                readyPosState = readyPos.turretFront;
                break;
            case turretFront:
                if (timer1.milliseconds() >= 300) {
                    turretServo.setPosition(0.85);
                    timer1.reset();
                    readyPosState = readyPos.fourBarClawOpen;
                }
                break;
            case fourBarClawOpen:
                if (timer1.milliseconds() >= 300) {
                    clawServo.setPosition(0.45);
                    horTargetPos = CHANGEABLEhorTargetPos;
                    if (Math.abs(lhArmMotor.getCurrentPosition() - CHANGEABLEhorTargetPos) < 60) {
                        count = 0;
                    }
                }
                break;


        }
    }
    public void outtakeCycle () {
        switch (outtakeState) {
            case vertExtend:
                count = 1;
                outtakeState = Outtake.readyPosFunc;
                break;
            case readyPosFunc:
                vertTargetPos = 560;
                outtakeState = Outtake.vertRetract;
                break;
            case vertRetract:
                if (Math.abs(vertMotor.getCurrentPosition() - vertTargetPos) < 20) {
                    vertTimer.reset();
                    outtakeState = Outtake.vertRetractP2;
                }
                break;
            case vertRetractP2:
                if (vertTimer.milliseconds() >= 250) {
                    vertTargetPos = 0;
                    if (Math.abs(vertMotor.getCurrentPosition() - 0) < 15) {
                        count2 = 0;
                    }
                }
                break;


        }
    }
    public void pickUpStackCycle () {
        switch (pickUpStackState) {
            case clawClose:
                clawServo.setPosition(0.7);
                sleep(150);
                pickUpStackState = pickUpStack.fourBarUp;
                break;
            case fourBarUp:
                pickUpStackState = pickUpStack.horRetract;
                break;
            case horRetract:
                leftFourBar.setPosition(0.6);
                rightFourBar.setPosition(0.6);
                sleep(500);
                horTargetPos = 200;
                pickUpStackState = pickUpStack.fourBarBack;
                break;
            case fourBarBack:
                if (Math.abs(lhArmMotor.getCurrentPosition() - 200) < 45) {
                    leftFourBar.setPosition(0.75);
                    rightFourBar.setPosition(0.75);
                    sleep(150);
                    turretServo.setPosition(0.12);
                    horTargetPos = -75;
                    pickUpStackState = pickUpStack.turretBack;
                }
                break;
            case turretBack:
                pickUpStackState = pickUpStack.fourBarOuttake;
                break;
            case fourBarOuttake:
                telemetry.addLine();
                telemetry.addData("hor ticks",lhArmMotor.getCurrentPosition());
                if (Math.abs(lhArmMotor.getCurrentPosition() - 0) < 15) {
                    sleep(200);
                    leftFourBar.setPosition(backPos);
                    rightFourBar.setPosition(backPos);
                    pickUpStackState = pickUpStack.clawOpen;
                }
                break;
            case clawOpen:
                sleep(750);
                clawServo.setPosition(0.4);
                pickUpStackState = pickUpStack.fourBarReset;
                break;
            case fourBarReset:
                sleep(100);
                leftFourBar.setPosition(0.5);
                rightFourBar.setPosition(0.5);
                count3 = 0;
                break;


        }
    }

}
