package org.firstinspires.ftc.teamcode.autonomous.fourBar;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.teleopstrafer;

@Config
@Autonomous(name="FSM4bar")
public class fourbaraxonFSM extends LinearOpMode {

    ServoImplEx leftFourBar;
    ServoImplEx rightFourBar;
    Servo clawServo;
    Servo turretServo;
    public static double turretPos;
    public static double clawPos;

    public enum fourBarCycle {
        cycleStart,
        fourBarSet,
        turretFrontClawClose,
        fourBarClawIntake,
        fourBarReset,
        turretBack,
        fourBarBack,
        clawOpen,
        //4barback
    }

    fourBarCycle fourBarCycleState = fourBarCycle.cycleStart;
    ElapsedTime timer1= new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        leftFourBar = hardwareMap.get(ServoImplEx.class, "leftFourBar");
        rightFourBar = hardwareMap.get(ServoImplEx.class, "rightFourBar");
        clawServo = hardwareMap.get(ServoImplEx.class, "clawServo");
        turretServo = hardwareMap.get(ServoImplEx.class, "turretServo");

        leftFourBar.setDirection(Servo.Direction.REVERSE);

        leftFourBar.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightFourBar.setPwmRange(new PwmControl.PwmRange(500, 2500));

        /*
        Claw facing backwards position: 0.15
        Claw facing outwards position: 0.825

        Claw open: 0.4
        Claw close: 0.62
         */


        waitForStart();

        while (opModeIsActive()) {
            CycleP1();


            break;

        }
    }

    public void CycleP1() {
        switch (fourBarCycleState) {
            case cycleStart:
                leftFourBar.setPosition(0.75);
                rightFourBar.setPosition(0.75);
                fourBarCycleState = fourBarCycle.fourBarSet;
            case fourBarSet:
                clawServo.setPosition(0.4);
                sleep(300);
                turretServo.setPosition(0.15);
                sleep(150);
                fourBarCycleState = fourBarCycle.turretFrontClawClose;
            case turretFrontClawClose:
                clawServo.setPosition(0.62);
                sleep(500);
                leftFourBar.setPosition(0.87);
                rightFourBar.setPosition(0.87);
                fourBarCycleState = fourBarCycle.fourBarClawIntake;
            case fourBarClawIntake:
                sleep(300);
                leftFourBar.setPosition(0.75);
                rightFourBar.setPosition(0.75);
                fourBarCycleState = fourBarCycle.fourBarReset;
            case fourBarReset:
                sleep(300);
                turretServo.setPosition(0.15);
                sleep(150);
                fourBarCycleState = fourBarCycle.turretBack;
            case turretBack:
                leftFourBar.setPosition(0.35);
                rightFourBar.setPosition(0.35);
                fourBarCycleState = fourBarCycle.fourBarBack;
            case fourBarBack:
                sleep(400);
                clawServo.setPosition(0.4);
                fourBarCycleState = fourBarCycle.clawOpen;
            case clawOpen:
                sleep(200);
                leftFourBar.setPosition(0.5);
                rightFourBar.setPosition(0.5);












        }
    }
}
