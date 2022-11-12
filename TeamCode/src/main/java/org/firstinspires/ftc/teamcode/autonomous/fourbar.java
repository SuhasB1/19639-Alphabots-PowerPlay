package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="fourbartest", group="Training")
public class fourbar extends LinearOpMode {
    public Servo fourBarServo;
    public DcMotorEx motor1;
    public DcMotorEx motor2;
    @Override
    public void runOpMode() {
        fourBarServo = hardwareMap.servo.get("fourBarServo");
        fourBarServo.setDirection(Servo.Direction.REVERSE);
        motor1 = hardwareMap.get(DcMotorEx.class,"lhArmMotor");
        motor2 = hardwareMap.get(DcMotorEx.class, "rhArmMotor");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Servo Pos:", fourBarServo.getPosition());
            telemetry.update();
            motor1.setPower(0);
            motor2.setPower(0);
            if(gamepad1.a){
                fourBarServo.setPosition(0.5);
            }
            if(gamepad1.b){
                fourBarServo.setPosition(0.1);
            }
        }
    }
}
