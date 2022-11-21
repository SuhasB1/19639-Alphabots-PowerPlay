package org.firstinspires.ftc.teamcode.anandrandomtestingshit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class functiontesting extends LinearOpMode {

    DcMotor fl;
    DcMotor fr;
    DcMotor br;
    DcMotor bl;


    @Override
    public void runOpMode()
    {


    }



    public void initialization()
    {
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");
    }
}
