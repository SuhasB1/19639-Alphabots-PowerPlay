package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="auto", group="Training")
public class armtest extends LinearOpMode {
    autofunctionlibrary robot = new autofunctionlibrary();
    public ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        DcMotor lhArmMotor;
        lhArmMotor = hardwareMap.dcMotor.get("lh_arm_motor");
        lhArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive()) {
            lhArmMotor.setPower(0.2);
                if(getRuntime() == 2.0){
                    lhArmMotor.setPower(0);
                }
            break;
        }
    }
}
