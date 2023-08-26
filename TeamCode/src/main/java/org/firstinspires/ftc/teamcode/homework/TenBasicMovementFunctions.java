package org.firstinspires.ftc.teamcode.homework;
/*
IMPORTANT: Before you start create a copy of this program. DO NOT EDIT THIS CURRENT PROGRAM.
Create a copy by right clicking the program name, then press copy.
Right click the program name again and press paste. Once you complete this, upload your new program to github
When copying the program, make sure to edit the program name by adding ur name at the start.
Ex: "Suhas_TenBasicMovementFunctions"
*/

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//Make sure you have read the comments above before moving on!!
// You will need more imports to get this program to work
//To import a function, right click on any red underline you see
//Press Show Context Actions
//Press import class and it should automatically add it here.

public class TenBasicMovementFunctions extends OpMode {

    DcMotor enterDcMotorName1;
    DcMotor enterDcMotorName2;
    DcMotor enterDcMotorName3;
    DcMotor enterDcMotorName4;
    //here is an example of how to import a DcMotor
    //Using this, define all four wheels name.
    //the names should be leftWheel, rightWheel, frontLeftWheel, and frontRightWheel

    @Override
    public void init() { // here is the init function. This is required for any initialization processes
        //It just means what you want the robot to do whenever it is ready to start, but not yet started.
        enterDcMotorName1 = hardwareMap.dcMotor.get("leftFront");
        //here is an example of the hardware mapping.
        //using the DcMotor names you created above, add them before the equal sign
       //keep the names that are in green within the .get(). dont change them.
        enterDcMotorName2 = hardwareMap.dcMotor.get("rightFront");
        enterDcMotorName3 = hardwareMap.dcMotor.get("rightRear");
        enterDcMotorName4 = hardwareMap.dcMotor.get("leftRear");



        enterDcMotorName1.setDirection(DcMotorSimple.Direction.REVERSE);
        //here is an example for how to reverse a motor.
        //you will need to reverse ALL motors EXCEPT for the wheel on the back right.


        enterDcMotorName1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        enterDcMotorName2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        enterDcMotorName3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        enterDcMotorName4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //remainder: delete the enterDcMotorName variables and add whatever variable names you created at the start.
        //all this does is it makes it so when you set the power to zero,
        //instead of slowing to a stop, it immediately stops.
    }

    @Override
    public void loop() {
    }
    //here is your loop function. This infinitely runs whatever is inside
    //while the program is running
    //for this exercise, leave it blank, we aren't going to use it.

    @Override
    public void start(){
        //this is where you run your functions. for now leave it blank, we will change it at the meeting
    }

    public void enterFunctionName(){
        //this is how you create a function.
        //create a function for each type of movement with a different name
        //as an example I will show you how to move forward
        enterDcMotorName1.setPower(0.5);
        enterDcMotorName2.setPower(0.5);
        enterDcMotorName3.setPower(0.5);
        enterDcMotorName4.setPower(0.5);
        //if all motors are moving forward at a speed of 0.5, the robot will move forward.
        //if the chart includes a wheel moving backwards, use "motorName.setPower(-0.5);"
        //this is really basic and will run this forever, but is a good starting point
        //message me on discord if u need help or the instructions are unclear.
        //the chart sent in discord will help you see all 10 basic movements
        //You should have a total of 10 functions created at the very end
        //complete this by the Sunday meeting.
    }

}
