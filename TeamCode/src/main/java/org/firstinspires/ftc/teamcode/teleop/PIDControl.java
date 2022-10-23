package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.autofunctionlibrary;

@Autonomous(name="PIDControl", group="Training")
public class PIDControl extends LinearOpMode {
    autofunctionlibrary robot = new autofunctionlibrary();
    DcMotorEx motor;

    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double Kg = 0;

    double integralSum = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class,"lhArmMotor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            double power = PIDController(100, motor.getCurrentPosition());
            motor.setPower(power);
            break;
        }
    }

    public double PIDController(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + Kg;
        return output;
    }
}




