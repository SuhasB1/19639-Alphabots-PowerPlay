package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;

@Config

public class PIDControl extends LinearOpMode {
    DcMotorEx motor1;
    DcMotorEx motor2;

    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kg = 0; //gravity feedforward constant (not needed for hor slides)
    public static double targetPosition = 100;
    public static double maxMotorTicks = 650;
    double integralSum = 0;
    public static double pos = 0.4;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    double power = 0;
    double power2 = 0;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotorEx.class,"rhArmMotor");
        motor2 = hardwareMap.get(DcMotorEx.class, "lhArmMotor");
        Servo fourBarServo = hardwareMap.servo.get("fourBarServo");
        Servo fourBarServo2 = hardwareMap.servo.get("fourBarServo2");
        fourBarServo.setDirection(Servo.Direction.REVERSE);
        fourBarServo.setPosition(pos);
        fourBarServo2.setPosition(pos);
        fourBarServo.getController().pwmEnable();
        fourBarServo2.getController().pwmEnable();
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        dashboard.setTelemetryTransmissionInterval(25);
        TelemetryPacket packet = new TelemetryPacket();


        waitForStart();
        packet.put("power", power);
        packet.put("positionLH", motor2.getCurrentPosition());
        packet.put("positionRH", motor1.getCurrentPosition());
        packet.put("targetPosition", targetPosition);
        dashboard.sendTelemetryPacket(packet);
        sleep(5000);
        while (opModeIsActive()) {
                power = PIDController(targetPosition, motor1.getCurrentPosition());
                power2 = PIDController(targetPosition, motor2.getCurrentPosition());
                motor1.setPower(power);
                motor2.setPower(power2);

            packet.put("power", power);
            packet.put("positionLH", motor2.getCurrentPosition());
            packet.put("positionRH", motor1.getCurrentPosition());
            packet.put("targetPosition", targetPosition);
            dashboard.sendTelemetryPacket(packet);
            }
        }

    public double PIDController(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + Kg;
        if ( error > 5 || error <  -5){
            return output;
        }
        else{
            output = 0;
            return output;
        }
    }
}




