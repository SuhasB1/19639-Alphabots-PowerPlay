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

@Config
@Autonomous (name="horizontalpid")
public class RRHorPID extends LinearOpMode {
    DcMotor rhArmMotor;
    DcMotor lhArmMotor;


    public static double kP = 0.0023;
    public static double kD = 0.000015;
    public static double kI = 0;
    public static double targetPos = 0;
    public static double horSpeed = 0.7;
    PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);
    private final FtcDashboard dashboard = FtcDashboard.getInstance();




    @Override
    public void runOpMode() throws InterruptedException {

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