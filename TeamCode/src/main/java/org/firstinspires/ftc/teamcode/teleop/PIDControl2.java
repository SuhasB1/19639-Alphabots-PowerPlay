package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name="PIDControl2", group="Training")
public class PIDControl2 extends LinearOpMode {
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
        motor2 = hardwareMap.get(DcMotorEx.class, "lhArmMotor");
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        while (opModeIsActive()) {
                motor2.setPower(0);
                // double power = PIDController(targetPosition, motor1.getCurrentPosition());


              //  packet.put("power", liftPower);
              //  packet.put("positionLH", motor2.getCurrentPosition());
               // packet.put("positionRH", motor1.getCurrentPosition());
               // packet.put("error", lastError);
                //packet.put("output", PIDController(targetPosition, motor1.getCurrentPosition()));
               // packet.put("targetPosition", targetPosition);

                // motor1.setPower(power);

                //dashboard.sendTelemetryPacket(packet);
            updateLiftDrive();
            }

        }
    public void updateLiftDrive(){

        double currentLiftPosition = motor1.getCurrentPosition();  // Get the position of the lift in ticks
        TelemetryPacket packet = new TelemetryPacket();

        dashboard.setTelemetryTransmissionInterval(25);

        double liftError = targetPosition - currentLiftPosition;  // determine the error in ticks between our current lift position and where it should be
        double liftPower = 0;

        if (liftError > 5 || liftError < -5){              // Hysteresis band to provide damping/braking zone to reduce oscillations

            liftPower = ((liftError * .03)+ .1);   		// PID with P only factor plus feed forward

            if (liftPower > .4){
                liftPower = .4;
            }
            if (liftPower < -.4){
                liftPower = -.4;
            }
        }
        else {

             liftPower = 0;

        } // end of if-else
        packet.put("power", liftPower);
        packet.put("positionLH", motor2.getCurrentPosition());
        packet.put("positionRH", motor1.getCurrentPosition());
        packet.put("targetPosition", targetPosition);
        dashboard.sendTelemetryPacket(packet);


        motor1.setPower(liftPower);


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




