package org.firstinspires.ftc.teamcode.homework;
/*
IMPORTANT: Before you start create a copy of this program. DO NOT EDIT THIS CURRENT PROGRAM.
Create a copy by right clicking the program name, then press copy.
Right click the program name again and press paste. Once you complete this, upload your new program to github
When copying the program, make sure to edit the program name by adding ur name at the start.
Ex: "Suhas_TenBasicMovementFunctions"
*/

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//Make sure you have read the comments above before moving on!!
// You will need more imports to get this program to work
//To import a function, right click on any red underline you see
//Press Show Context Actions
//Press import class and it should automatically add it here.
@Autonomous(name="basic movement")
public class TenBasicMovementFunctions extends OpMode {

    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor frontLeftWheel;
    DcMotor frontRightWheel;

    @Override
    public void init() {
        frontLeftWheel = hardwareMap.dcMotor.get("leftFront");
        frontRightWheel = hardwareMap.dcMotor.get("rightFront");
        rightWheel = hardwareMap.dcMotor.get("rightRear");
        leftWheel = hardwareMap.dcMotor.get("leftRear");



        frontLeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftWheel.setDirection(DcMotorSimple.Direction.REVERSE);


        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
    }

    @Override
    public void start(){
        moveForward(500);
        backward(500);
    }

    public void moveForward(int time){
        frontLeftWheel.setPower(0.5);
        frontRightWheel.setPower(0.5);
        rightWheel.setPower(0.5);
        leftWheel.setPower(0.5);
        sleep(time);
        stopAllMotors();
    }

    public void strafeLeft(int time){
        frontLeftWheel.setPower(-0.5);
        leftWheel.setPower(0.5);
        frontRightWheel.setPower(0.5);
        rightWheel.setPower(-0.5);
        sleep(time);
        stopAllMotors();
    }

    public void strafeRight(int time){
        frontLeftWheel.setPower(0.5);
        leftWheel.setPower(-0.5);
        rightWheel.setPower(0.5);
        frontRightWheel.setPower(-0.5);
        sleep(time);
        stopAllMotors();
    }

    public void backward(int time){
        frontRightWheel.setPower(-0.5);
        rightWheel.setPower(-0.5);
        frontLeftWheel.setPower(-0.5);
        leftWheel.setPower(-0.5);
        sleep(time);
        stopAllMotors();
    }

    public void turnLeft(int time){
        frontLeftWheel.setPower(-0.5);
        leftWheel.setPower(-0.5);
        frontRightWheel.setPower(0.5);
        rightWheel.setPower(0.5);
        sleep(time);
        stopAllMotors();
    }
    public void turnRight(int time){
        frontLeftWheel.setPower(0.5);
        leftWheel.setPower(0.5);
        frontRightWheel.setPower(-0.5);
        rightWheel.setPower(-0.5);
        sleep(time);
        stopAllMotors();
    }

    public void diagonalTopLeft(int time){
        frontRightWheel.setPower(0.5);
        leftWheel.setPower(0.5);
        sleep(time);
        stopAllMotors();
    }


    public void diagonalBottomLeft(int time){
        frontLeftWheel.setPower(-0.5);
        rightWheel.setPower(-0.5);
        sleep(time);
        stopAllMotors();
    }


    public void diagonalTopRight(int time){
        frontLeftWheel.setPower(0.5);
        rightWheel.setPower(0.5);
        sleep(time);
        stopAllMotors();
    }



    public void diagonalBottomRight(int time){
        frontRightWheel.setPower(-0.5);
        leftWheel.setPower(-0.5);
        sleep(time);
        stopAllMotors();
    }

    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    public void stopAllMotors(){
        frontLeftWheel.setPower(0);
        frontRightWheel.setPower(0);
        rightWheel.setPower(0);
        leftWheel.setPower(0);
    }
}
