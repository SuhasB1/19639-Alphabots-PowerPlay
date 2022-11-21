package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="autoarmtest", group="Training")
public class armtest extends LinearOpMode {

    DcMotor lhArmMotor;
    DcMotor rhArmMotor;


    @Override
    public void runOpMode() {

        lhArmMotor = hardwareMap.dcMotor.get("lhArmMotor");
        lhArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lhArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rhArmMotor = hardwareMap.dcMotor.get("rhArmMotor");
        rhArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rhArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rhArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lhArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Ticks:", lhArmMotor.getCurrentPosition());
            telemetry.addData("Ticks:", rhArmMotor.getCurrentPosition());

            telemetry.update();
        }
    }
    public void encoders() {
        final double ENCODER_TPR = 537.6;

    }
}
