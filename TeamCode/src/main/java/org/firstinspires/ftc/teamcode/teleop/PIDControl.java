package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
    double Kg = 0; //gravity feedforward constant (not needed for hor slides)
    double targetPosition = 0;

    double integralSum = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class,"lhArmMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        TelemetryPacket packet = new TelemetryPacket();

        dashboard.setTelemetryTransmissionInterval(25);

        waitForStart();
        while (opModeIsActive()) {
            double power = PIDController(targetPosition, motor.getCurrentPosition());

            packet.put("power", power);
            packet.put("position", motor.getCurrentPosition());
            packet.put("error", lastError);
            packet.put("targetPosition", targetPosition);

            motor.setPower(power);

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




