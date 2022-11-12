package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;

@Config
@Autonomous(name="PIDControl", group="Training")
public class PIDControl extends LinearOpMode {
    DcMotorEx motor1;
    DcMotorEx motor2;

    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kg = 0; //gravity feedforward constant (not needed for hor slides)
    public static double targetPosition = 0;
    public static double maxMotorTicks = 650;

    double integralSum = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotorEx.class,"lhArmMotor");
        motor2 = hardwareMap.get(DcMotorEx.class, "rhArmMotor");
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        TelemetryPacket packet = new TelemetryPacket();

        dashboard.setTelemetryTransmissionInterval(25);

        waitForStart();
        while (opModeIsActive()) {
            double power = PIDController(targetPosition, (motor1.getCurrentPosition() + motor2.getCurrentPosition()) / 2);
            if (motor1.getCurrentPosition() > maxMotorTicks|| motor1.getCurrentPosition() < 0){
                motor1.setPower(0);
                motor2.setPower(0);
            }

            packet.put("power", power);
            packet.put("positionLH", motor1.getCurrentPosition());
            packet.put("positionRH", motor2.getCurrentPosition());
            packet.put("error", lastError);
            packet.put("targetPosition", targetPosition);

            motor1.setPower(power);
            motor2.setPower(power);

            dashboard.sendTelemetryPacket(packet);
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




