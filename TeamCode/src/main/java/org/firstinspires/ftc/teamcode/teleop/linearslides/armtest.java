package org.firstinspires.ftc.teamcode.teleop.linearslides;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous (name="horizontal_tick_counter")
public class armtest extends LinearOpMode {

    DcMotor lhArmMotor;
    DcMotor rhArmMotor;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();



    @Override
    public void runOpMode() {

        lhArmMotor = hardwareMap.dcMotor.get("lhArmMotor");
        lhArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lhArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rhArmMotor = hardwareMap.dcMotor.get("rhArmMotor");
        rhArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rhArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rhArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        TelemetryPacket packet = new TelemetryPacket();
        dashboard.setTelemetryTransmissionInterval(25);


        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Ticks:", lhArmMotor.getCurrentPosition());
            telemetry.addData("Ticks:", rhArmMotor.getCurrentPosition());
            packet.put("left position", lhArmMotor.getCurrentPosition());
            packet.put("right position", rhArmMotor.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
        }
    }
    public void encoders() {
        final double ENCODER_TPR = 537.6;

    }
}
