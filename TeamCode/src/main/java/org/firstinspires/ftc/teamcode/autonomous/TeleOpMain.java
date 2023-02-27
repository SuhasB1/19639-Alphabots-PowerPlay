package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="MainTeleOp")
public class TeleOpMain extends LinearOpMode {

    ServoImplEx leftFourBar;
    ServoImplEx rightFourBar;
    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;
    Servo clawServo;
    Servo turretServo;
    DcMotor vertMotor;
    DcMotorEx lhArmMotor;
    DcMotorEx rhArmMotor;
    int count = 0;
    int count2 = 0;
    int count3 = 0;
    int count4 = 0;
    PIDCoefficients vertCoeffs = new PIDCoefficients(-0.03, 0, -0.00025);
    PIDCoefficients horCoeffs = new PIDCoefficients(0.0041, 0, 0.00001);
    public static double vertTargetPos = 0;
    public static double horTargetPos = 0;
    double vertvertTargetPos = 0;
    double horhorTargetPos = 0;
    public static double turretPos = 0.12;
    public static double vertDeadband = 20;
    public static double backPos = 0.372;
    public static double downPos = 0.88;
    public static double coneInterval = 0;
    public final double kG = 0.05;
    public static double wait = 250;
    public double coneCount = 0;
    double horSpeed = 0.75;
    public double multiplier = 1;

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
    public enum Outtake {
        vertExtend,
        readyPosFunc,
        vertRetract,
        vertRetractP2
    }
    public enum horExtend {
        horExtended,
        fourBarDown
    }

    readyPos readyPosState = readyPos.fourBarClawClose;
    pickUp pickUpState = pickUp.horExtend;
    Outtake outtakeState = Outtake.vertExtend;
    horExtend horExtendState = horExtend.horExtended;

    ElapsedTime timer1= new ElapsedTime();
    ElapsedTime testTimer= new ElapsedTime();
    ElapsedTime vertTimer= new ElapsedTime();
    int test = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        leftFourBar = hardwareMap.get(ServoImplEx.class, "leftFourBar");
        rightFourBar = hardwareMap.get(ServoImplEx.class, "rightFourBar");
        clawServo = hardwareMap.get(ServoImplEx.class, "clawServo");
        turretServo = hardwareMap.get(ServoImplEx.class, "turretServo");
        leftFourBar.setDirection(Servo.Direction.REVERSE);
        leftFourBar.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightFourBar.setPwmRange(new PwmControl.PwmRange(500, 2500));
        leftWheel = hardwareMap.get(DcMotorEx.class, "leftFront");
        backLeftWheel = hardwareMap.get(DcMotorEx.class, "leftRear");
        backRightWheel = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightWheel = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftWheel.setDirection(DcMotorSimple.Direction.FORWARD); //leftWheel
        backLeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightWheel.setDirection(DcMotorSimple.Direction.FORWARD); //
        rightWheel.setDirection(DcMotorSimple.Direction.REVERSE);


        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  //setting it so when power is 0, robot stops.

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
        count2 = 0;
        count3 = 0;
        count4 = 0;
        test = 0;
        coneCount = 0;

        leftFourBar.setPosition(0.5);
        rightFourBar.setPosition(0.5);

        waitForStart();

        while (opModeIsActive()) {
            vertController.setTargetPosition(vertTargetPos);
            horController.setTargetPosition(horTargetPos);
            vertSlidesPID(vertController);
            horSlidesPID(horController);
            moveDriveTrain();
            telemetry.update();
            if(gamepad1.square){
                multiplier = 0.3;
            }
            if(gamepad1.triangle){
                multiplier = 0.7;
            }
            if(gamepad1.circle){
                multiplier = 1;
            }
            if(count == 1){
                readyPosCycle();
            }
            if(count2 == 1){
                outtakeCycle();
            }
            if(count3 == 1){
                pickUpCycle();
            }
            if (count4 == 1){
                horExtension();
            }
            if(gamepad2.triangle){
                vertvertTargetPos = 860;
                readyPosState = readyPos.fourBarClawClose;
                outtakeState = Outtake.vertExtend;
                count2 = 1;
                //vert high
            }
            if(gamepad2.square){
                vertvertTargetPos = 560;
                readyPosState = readyPos.fourBarClawClose;
                outtakeState = Outtake.vertExtend;
                count2 = 1;
                //vert mid
            }
            if(gamepad2.circle){
                leftFourBar.setPosition(0.5);
                rightFourBar.setPosition(0.5);
            }
            if(gamepad2.cross){
                leftFourBar.setPosition(downPos);
                rightFourBar.setPosition(downPos);
                //vert 0
            }
            if(gamepad2.dpad_up){
                horhorTargetPos = 640;
                horExtendState = horExtend.horExtended;
                count4 = 1;
                //hor far
            }

            if(gamepad2.dpad_right){
                readyPosState = readyPos.fourBarClawClose;
                count = 1;
                //hor small
            }
            if(gamepad2.dpad_left){
                horhorTargetPos = 550;
                horExtendState = horExtend.horExtended;
                count4 = 1;
               //hor mid

            }
            if(gamepad2.dpad_down){
                pickUpState = pickUp.horExtend;
                count3 = 1;
                //hor back
            }
            if(gamepad2.left_bumper){
                // main cycle p1

            }
            if(gamepad2.right_bumper){
                //main cycle p2
            }



        }
    }
    public void moveDriveTrain() {
        double driveForward = 0; //Moves forwards and backwards
        double driveBackward = 0;
        double strafe = 0; //Move side-to-side
        double rotate = 0;
        double drive = 0;
        double denominator = 1;

        drive = gamepad1.left_stick_y;
        strafe = gamepad1.right_stick_x * 1.1;
        rotate = gamepad1.left_stick_x;

        driveForward = gamepad1.right_trigger;
        driveBackward = gamepad1.left_trigger;

        denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(rotate), 1);
        if(driveForward > 0) {
            rightWheel.setPower(-driveForward * (multiplier));
            backRightWheel.setPower(-driveForward * multiplier);
            leftWheel.setPower(-driveForward *multiplier);
            backLeftWheel.setPower(-driveForward * multiplier);
        } else if(driveBackward > 0) {
            rightWheel.setPower(driveBackward * multiplier);
            backRightWheel.setPower(driveBackward * multiplier);
            leftWheel.setPower(driveBackward * multiplier);
            backLeftWheel.setPower(driveBackward * multiplier);

        } else {
            rightWheel.setPower(((drive + strafe  + rotate) / denominator) * multiplier);
            backRightWheel.setPower(((drive - rotate + strafe) / denominator) * multiplier);
            leftWheel.setPower(((drive - rotate - strafe) / denominator) * multiplier);
            backLeftWheel.setPower(((drive + rotate - strafe) / denominator) * multiplier);
        }
    }
    public void vertSlidesPID(PIDFController controller){
        double correction = controller.update(vertMotor.getCurrentPosition());
        vertMotor.setPower(correction + kG);
    }
    public void horSlidesPID(PIDFController controller)
    {
        double correction = controller.update((lhArmMotor.getCurrentPosition() + rhArmMotor.getCurrentPosition())/2);
        if (correction > horSpeed){
            correction = horSpeed;
        }
        if (correction < -horSpeed){
            correction = -horSpeed;
        }
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
                    turretServo.setPosition(0.85);
                    timer1.reset();
                    telemetry.addData("2", testTimer.milliseconds());
                    readyPosState = readyPos.fourBarClawOpen;
                }
                break;

            case fourBarClawOpen:
                if (timer1.milliseconds() >= 400) {
                    leftFourBar.setPosition(0.75);
                    rightFourBar.setPosition(0.75);
                    clawServo.setPosition(0.4);
                    count = 0;

                }
                break;



        }
    }

    public void horExtension(){
        switch(horExtendState) {
            case horExtended:
            leftFourBar.setPosition(downPos);
            rightFourBar.setPosition(downPos);
            sleep(200);
            clawServo.setPosition(0.4);
            sleep(400);
            horSpeed = 0.7;
            horTargetPos = horhorTargetPos;
            horExtendState = horExtend.fourBarDown;
            break;
            case fourBarDown:
                if (Math.abs(lhArmMotor.getCurrentPosition() - horhorTargetPos) < 20) {
                    count4 = 0;
                }
            break;
        }
    }

    public void pickUpCycle(){
        switch (pickUpState) {
            case horExtend:
                clawServo.setPosition(0.6);
                sleep(100);
                pickUpState = pickUp.horRetract;
                break;
            case horRetract:
                horSpeed = 1;
                horTargetPos = -20;
                pickUpState = pickUp.fourBarTurretBack;
                break;
            case fourBarTurretBack:
                leftFourBar.setPosition(0.75);
                rightFourBar.setPosition(0.75);
                turretServo.setPosition(turretPos);
                testTimer.reset();
                pickUpState = pickUp.fourBarOuttake;
                break;
              case fourBarOuttake:
                  if (Math.abs(lhArmMotor.getCurrentPosition() - 0) < 15) {
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
                sleep(700);
                clawServo.setPosition(0.4);
                pickUpState = pickUp.fourBarReset;
                break;
            case fourBarReset:
                sleep(300);
                leftFourBar.setPosition(0.5);
                rightFourBar.setPosition(0.5);
                count3 = 0;
                break;

        }
    }
    public void outtakeCycle(){
        switch(outtakeState){
            case vertExtend:
                vertTargetPos = vertvertTargetPos; //860 for high
                outtakeState = Outtake.readyPosFunc;
                break;
            case readyPosFunc:
                count =1;
                outtakeState = Outtake.vertRetract;
                break;
            case vertRetract:
                if (Math.abs(vertMotor.getCurrentPosition() - vertvertTargetPos) < 20) {
                    telemetry.addData("ticks", vertMotor.getCurrentPosition());
                    vertTimer.reset();
                    telemetry.addData("time", vertTimer.milliseconds());
                    outtakeState = Outtake.vertRetractP2;
                }
                break;
            case vertRetractP2:
                if (vertTimer.milliseconds() >= wait) {
                    vertTargetPos = 0;
                    count2 = 0;
                }
                break;


        }
    }

}

