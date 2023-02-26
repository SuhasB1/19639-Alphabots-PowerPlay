package org.firstinspires.ftc.teamcode.teleop.linearslides;

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
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.control.PIDCoefficients;


@Config
@Autonomous (name="RRvertpid")
public class RRVertPID extends LinearOpMode {
    DcMotorEx motor1;



    public static double kD = -0.00025;
    public static double kP = -0.03;
    public static double kG = -0.1;
    final public double kI = 0;
    public static double targetPos = 0; //860 (high)
    public static double maxMotorTicks = 2000;
    double maxVertPower = 0.7;
    double derivative;
    double lastError;
    double integralSum = 0;
    public ElapsedTime runtime = new ElapsedTime();

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    // specify coefficients/gains
    PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);
    // create the controller
    @Override
    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotorEx.class,"vertMotor");
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFController controller = new PIDFController(coeffs);



        waitForStart();
        TelemetryPacket packet = new TelemetryPacket();
        dashboard.setTelemetryTransmissionInterval(25);
        packet.put("position", motor1.getCurrentPosition());
        packet.put("targetPosition", targetPos);
        dashboard.sendTelemetryPacket(packet);
        sleep(5000);
        controller.setTargetPosition(targetPos);
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
            telemetry.addData("ticks" , motor1.getCurrentPosition());
            telemetry.update();
            dashboard.setTelemetryTransmissionInterval(25);
            packet.put("position", motor1.getCurrentPosition());
            packet.put("targetPosition", targetPos);
            dashboard.sendTelemetryPacket(packet);

            vertSlidesPID(controller);
            //860 high ticks, 560 mid, 900 physical max



        }

    }
    public void vertSlidesPID(PIDFController controller){
        double correction = controller.update(motor1.getCurrentPosition());
        motor1.setPower(correction + kG);
    }
}