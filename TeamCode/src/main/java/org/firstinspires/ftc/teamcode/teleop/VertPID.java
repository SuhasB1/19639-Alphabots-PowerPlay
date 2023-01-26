package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous (name="vertpid")
public class VertPID extends LinearOpMode {
    DcMotorEx motor1;



    public static double kD;
    public static double kP = 0.03;
    public static double targetPos = 0;
    public static double kG = 0.1;
    public static double maxMotorTicks = 2000;
    double maxVertPower = 0.7;
    double derivative;
    double lastError;
    double integralSum = 0;
    public ElapsedTime runtime = new ElapsedTime();

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotorEx.class,"vertMotor");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        while (opModeIsActive()) {

                // double power = PIDController(targetPosition, motor1.getCurrentPosition());


              //  packet.put("power", liftPower);
              //  packet.put("positionLH", motor2.getCurrentPosition());
               // packet.put("positionRH", motor1.getCurrentPosition());
               // packet.put("error", lastError);
                //packet.put("output", PIDController(targetPosition, motor1.getCurrentPosition()));
               // packet.put("targetPosition", targetPosition);

                // motor1.setPower(power);

                //dashboard.sendTelemetryPacket(packet);

            sleep(5000);
            telemetry.addData("ticks rh" , motor1.getCurrentPosition());

            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            dashboard.setTelemetryTransmissionInterval(25);

            packet.put("positionRH", motor1.getCurrentPosition());
            packet.put("targetPosition", targetPos);
            packet.put("error", lastError);
            dashboard.sendTelemetryPacket(packet);

            vertSlidesPID(targetPos);


            }

        }
    public void vertSlidesPID(double pos){
        runtime.startTime();
        double currentLiftPosition = motor1.getCurrentPosition();  // Get the position of the lift in ticks

        double liftError = targetPos - currentLiftPosition;  // determine the error in ticks between our current lift position and where it should be
        double liftPower = 0;
        derivative = (liftError - lastError) / runtime.seconds();
        lastError = liftError;

        runtime.reset();



        if (liftError > 10 || liftError < -10){// Hysteresis band to provide damping/braking zone to reduce oscillations

            liftPower = ((liftError * kP)  + (derivative * kD) +  kG); // PID with P and D factor plus feed forward

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


        motor1.setPower(liftPower);


    }


    }
    /*
    public double PIDController(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + Kg;
        return output;
    }tested not final stuff
    */




