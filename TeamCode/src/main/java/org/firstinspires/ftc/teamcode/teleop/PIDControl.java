package org.firstinspires.ftc.teamcode.teleop;

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
    public static double targetPosition = 100;
    public static double maxMotorTicks = 650;

    double integralSum = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotorEx.class,"rhArmMotor");
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dashboard.setTelemetryTransmissionInterval(25);


        waitForStart();
        sleep(5000);
        while (opModeIsActive()) {
                if(Math.abs(targetPosition-motor1.getCurrentPosition()) <= 5){
                    motor1.setPower(0);
                }
                double power = PIDController(targetPosition, motor1.getCurrentPosition());
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("positionRH", motor1.getCurrentPosition());
                packet.put("error", lastError);
                packet.put("output", PIDController(targetPosition, motor1.getCurrentPosition()));
                packet.put("targetPosition", targetPosition);

                motor1.setPower(power);

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
        return output;
    }
}




