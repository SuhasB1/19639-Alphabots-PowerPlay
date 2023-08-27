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

@TeleOp(name="TeleOpBasicMovement", group = "Training")
public class BasicMovement extends OpMode {
    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;

    @Override
    public void init(){
        // Assigning wheels to actual motors
        leftWheel = hardwareMap.get(DcMotor.class, "leftFront");
        backLeftWheel = hardwareMap.get(DcMotor.class, "leftRear");
        rightWheel = hardwareMap.get(DcMotor.class, "rightFront");
        backRightWheel = hardwareMap.get(DcMotor.class, "rightRear");

        // Setting direction
        leftWheel.setDirection(DcMotorSimple.Direction.FORWARD); //leftWheel
        backLeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightWheel.setDirection(DcMotorSimple.Direction.FORWARD); //
        rightWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        // if power 0 then brake
        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop(){
        moveDriveTrain();
    }
    double multiplier = 1;
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
            leftWheel.setPower(-driveForward * multiplier);
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

    
}
