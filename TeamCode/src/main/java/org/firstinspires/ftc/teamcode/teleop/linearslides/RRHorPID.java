package org.firstinspires.ftc.teamcode.teleop.linearslides;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.autonomous.TeleOpMain;

@Config
@Autonomous (name="horizontalpid")
public class RRHorPID extends LinearOpMode {
    DcMotor rhArmMotor;
    DcMotor lhArmMotor;
    ServoImplEx leftFourBar;
    ServoImplEx rightFourBar;


    public static double kP = 0.0045;
    public static double kD = 0.0000075;
    public static double kI = 0;
    public static double targetPos = 0;
    public static double horSpeed = 1;
    PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);
    private final FtcDashboard dashboard = FtcDashboard.getInstance();




    @Override
    public void runOpMode() throws InterruptedException {
        leftFourBar = hardwareMap.get(ServoImplEx.class, "leftFourBar");
        rightFourBar = hardwareMap.get(ServoImplEx.class, "rightFourBar");
        leftFourBar.setDirection(Servo.Direction.REVERSE);
        leftFourBar.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightFourBar.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rhArmMotor = hardwareMap.get(DcMotorEx.class,"rhArmMotor");
        lhArmMotor = hardwareMap.get(DcMotorEx.class, "lhArmMotor");
        rhArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lhArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rhArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lhArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rhArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lhArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rhArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFController controller = new PIDFController(coeffs);



        waitForStart();
        TelemetryPacket packet = new TelemetryPacket();
        dashboard.setTelemetryTransmissionInterval(25);
        leftFourBar.setPosition(0.5);
        rightFourBar.setPosition(0.5);
        packet.put("left position", lhArmMotor.getCurrentPosition());
        packet.put("targetPosition", targetPos);
        packet.put("right position", rhArmMotor.getCurrentPosition());
        packet.put("targetPosition", targetPos);
        dashboard.sendTelemetryPacket(packet);
        sleep(5000);
        controller.setTargetPosition(targetPos);
        waitForStart();
        while(opModeIsActive())
        {
            telemetry.addData("left ticks" , lhArmMotor.getCurrentPosition());
            telemetry.addData("right ticks" , rhArmMotor.getCurrentPosition());

            telemetry.update();
            dashboard.setTelemetryTransmissionInterval(25);
            packet.put("left position", lhArmMotor.getCurrentPosition());
            packet.put("right position", rhArmMotor.getCurrentPosition());
            packet.put("targetPosition", targetPos);
            dashboard.sendTelemetryPacket(packet);

            horizontalPID(controller);
        }


    }
    public void horizontalPID(PIDFController controller)
    {
        double correction = controller.update((lhArmMotor.getCurrentPosition() + rhArmMotor.getCurrentPosition())/2);
        if (correction > horSpeed){
            correction = horSpeed;
        }
        if (correction < -horSpeed){
            correction = -horSpeed;
        }
        lhArmMotor.setPower(correction);
        rhArmMotor.setPower(correction);
    }
}