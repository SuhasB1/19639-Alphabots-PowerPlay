package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@TeleOp(name="TeleOpStrafer", group="Training")
public class teleopstrafer extends OpMode {

        DcMotor leftWheel;
        DcMotor rightWheel;
        DcMotor backLeftWheel;
        DcMotor backRightWheel;
        DcMotor vertMotor;
        DcMotor rhArmMotor;
        DcMotor lhArmMotor;
        Servo fourBarServo;
        Servo fourBarServo2;
        CRServo intakeServo1;
        CRServo intakeServo2;

        public static double pos = 0.77;
        public static double pos2 = 0.335;
        public double targetPositionHor = 0;
        public double targetPositionVert = 0;
        public double multiplier = 1;
        boolean isSlowMode = false;
        double maxVertPower = 0.7;

        public enum CYCLE1 {
            CYCLE_START,
            HOR_EXTEND
        }
        public enum CYCLE2 {
            CYCLE_START,
            ARM_UP,
            HOR_RETRACT,
            ARM_SET,
            OUTTAKE,
            ARM_BACK,
            VERT_EXTEND,
            VERT_RETRACT

        }




        private ElapsedTime runtime= new ElapsedTime();
        ElapsedTime timer1= new ElapsedTime();
        ElapsedTime timer2= new ElapsedTime();
    CYCLE2 cycleState2 = CYCLE2.CYCLE_START;
    CYCLE1 cycleState1 = CYCLE1.CYCLE_START;

        @Override
        public void init() {
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

            vertMotor = hardwareMap.get(DcMotorEx.class,"vertMotor");
            rhArmMotor = hardwareMap.get(DcMotorEx.class,"rhArmMotor");
            lhArmMotor = hardwareMap.get(DcMotorEx.class, "lhArmMotor");

            vertMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rhArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lhArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            vertMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rhArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lhArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            vertMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rhArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lhArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            vertMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            rhArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            fourBarServo = hardwareMap.servo.get("fourBarServo");
            fourBarServo2 = hardwareMap.servo.get("fourBarServo2");
            intakeServo1 = hardwareMap.crservo.get("intakeServo1");
            intakeServo2 = hardwareMap.crservo.get("intakeServo2");

            intakeServo1.setDirection(CRServo.Direction.REVERSE);
            intakeServo2.setDirection(CRServo.Direction.REVERSE);

            fourBarServo.setDirection(Servo.Direction.REVERSE);
            fourBarServo.setPosition(0.5);
            fourBarServo2.setPosition(0.5);




        }

        @Override
        public void loop() {
            telemetry.addData("lh", lhArmMotor.getCurrentPosition());
            telemetry.addData("vert", vertMotor.getCurrentPosition());
            telemetry.addData("rh", rhArmMotor.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Is Slow Mode On?", isSlowMode);
            telemetry.update();
            moveDriveTrain();
            horSlidesPID(targetPositionHor);
            vertSlidesPID(targetPositionVert);

            if(gamepad2.triangle){
                maxVertPower = 0.7;
                vertSlidesMovement("high");
            }
            if(gamepad2.square){
                maxVertPower = 0.2;
                vertSlidesMovement("almost");
            }
            if(gamepad2.cross){
                maxVertPower = 0.5;
                vertSlidesMovement("none");
            }
            if(gamepad2.dpad_up){
                intakeServo1.setPower(0.5);
                intakeServo2.setPower(0.5);
                servoFuncLoop(pos);
                horSlidesMovement("half");

            }

            if(gamepad2.dpad_right){
                intakeServo1.setPower(0.5);
                intakeServo2.setPower(0.5);
                servoFuncLoop(pos);
                horSlidesMovement("almost");
            }
            if(gamepad2.dpad_left){
                intakeServo1.setPower(0.5);
                intakeServo2.setPower(0.5);
                servoFuncLoop(pos);
                horSlidesMovement("close");

            }
            if(gamepad2.dpad_down){
                horSlidesMovement("none");
                intakeServo1.setPower(0.5);
                intakeServo2.setPower(0.5);
                servoFuncLoop(0.3685);
            }
            if(gamepad2.left_bumper){
                fourBarServo.setPosition(0.875);
                fourBarServo2.setPosition(0.875);

            }
            if(gamepad2.right_bumper){
                servoOuttakeFunc(pos2);
            }
            if(gamepad2.right_stick_button){
                intakeServo1.setPower(-0.5);
                intakeServo2.setPower(-0.5);
            }
            if(gamepad2.left_stick_button){
                servoFuncLoop(pos + 0.05);
            }
            if(gamepad1.left_bumper){
                multiplier = 0.2;
                isSlowMode = true;
            }
            if(gamepad1.right_bumper){
                multiplier = 1;
                isSlowMode = false;
            }
            if(gamepad2.share){
                horSlidesMovement("none");
                intakeServo1.setPower(0.5);
                intakeServo2.setPower(0.5);
                servoFuncLoop(pos);
            }
            /*if(gamepad2.right_bumper){
                horSlidesMovement("full");
                sleep(2000);
                fourBarServo.setPosition(pos * 0.5);
                fourBarServo2.setPosition(pos * 0.5);
                fourBarServo.setPosition(pos * 0.75);
                fourBarServo2.setPosition(pos * 0.75);
                fourBarServo.setPosition(pos * 0.9);
                fourBarServo2.setPosition(pos * 0.9);
                fourBarServo.setPosition(pos);
                fourBarServo2.setPosition(pos);
                sleep(500);
                horSlidesMovement("none");
                sleep(2000);
                fourBarServo.setPosition(pos2 * 0.75);
                fourBarServo2.setPosition(pos2 * 0.75);
                fourBarServo.setPosition(pos2 * 0.9);
                fourBarServo2.setPosition(pos2 * 0.9);
                fourBarServo.setPosition(pos2);
                fourBarServo2.setPosition(pos2);
                sleep(500);
                vertSlidesMovement("high");



            }
            */

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
        public void sleep(int milliseconds) {
            try {
                Thread.sleep(milliseconds);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    public void vertSlidesMovement(String input){
            if(input.equals("high")){
                targetPositionVert = 2300;
            }
            else if(input.equals("almost")) {
                targetPositionVert = 2120; //find exact pos
            }
            else if(input.equals("none")){
                targetPositionVert = 0;
            }
    }
    public void horSlidesMovement(String input){
        if(input.equals("close")){
            targetPositionHor = 390;
        }
        else if(input.equals("half")) {
            targetPositionHor =334;
        }
        else if(input.equals("none")){
            targetPositionHor = 0;
        }
        else if(input.equals("almost")){
            targetPositionHor = 285;
        }
    }

    public void vertSlidesPID(double targetPosition){
        double currentLiftPosition = vertMotor.getCurrentPosition();  // Get the position of the lift in ticks

        double liftError = targetPosition - currentLiftPosition;  // determine the error in ticks between our current lift position and where it should be
        double liftPower = 0;

        if (liftError > 10 || liftError < -10){// Hysteresis band to provide damping/braking zone to reduce oscillations

            liftPower = ((liftError * 0.03)+ + .1); // PID with P and D factor plus feed forward

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
    public void horSlidesPID(double targetPosition){
        double currentLiftPosition = rhArmMotor.getCurrentPosition();  // Get the position of the lift in ticks

        double liftError = (targetPosition-5) - currentLiftPosition;  // determine the error in ticks between our current lift position and where it should be
        double liftPower = 0;
        double liftError2 = (targetPosition-5) - lhArmMotor.getCurrentPosition();
        double liftPower2 = 0;

        if (liftError > 5 || liftError < -5){// Hysteresis band to provide damping/braking zone to reduce oscillations

            liftPower = ((liftError * 0.005)); // PID with P and D factor plus feed forward

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
    public void CycleP1(){
            switch(cycleState1){
                case CYCLE_START:
                    if(gamepad1.left_bumper){
                        horSlidesMovement("half");
                        cycleState1 = CYCLE1.HOR_EXTEND;
                    }
                    break;
                case HOR_EXTEND:
                    if (Math.abs(lhArmMotor.getCurrentPosition() - 325) < 5 && Math.abs(rhArmMotor.getCurrentPosition() - 325) < 5){
                        servoFunc(pos);
                    }
                    break;
                default:
                    cycleState1 = CYCLE1.CYCLE_START;
            }
    }
    public void CycleP2(){
            switch (cycleState2){
                case CYCLE_START:
                    if(gamepad1.right_bumper) {
                        servoFunc(0.5);
                        cycleState2 = CYCLE2.ARM_UP;
                    }
                    break;
                case ARM_UP:
                    horSlidesMovement("none");
                    cycleState2 = CYCLE2.HOR_RETRACT;
                case HOR_RETRACT:
                    if (Math.abs(lhArmMotor.getCurrentPosition() - 0) < 5 && Math.abs(rhArmMotor.getCurrentPosition() - 0) < 5){
                        servoFunc(0.35);
                        cycleState2 = CYCLE2.ARM_SET;
                    }
                case ARM_SET:
                    sleep(1000);
                    intakeServo1.setPower(-0.2);
                    intakeServo2.setPower(-0.3);
                    cycleState2 = CYCLE2.OUTTAKE;
                case OUTTAKE:
                    sleep(1500);
                    fourBarServo.setPosition(0.5);
                    fourBarServo2.setPosition(0.5);
                    cycleState2 = CYCLE2.ARM_BACK;
                case ARM_BACK:
                    vertSlidesMovement("high");
                    cycleState2 = CYCLE2.VERT_EXTEND;
                case VERT_EXTEND:
                    if (Math.abs(vertMotor.getCurrentPosition() - 2300) < 5){
                        vertSlidesMovement("none");
                        cycleState2 = CYCLE2.VERT_RETRACT;
                    }
                case VERT_RETRACT:
                    if (Math.abs(vertMotor.getCurrentPosition() - 0) < 5){
                        telemetry.addLine();
                        telemetry.addData("Cycle Complete", targetPositionVert);
                    }











            }
    }



}
