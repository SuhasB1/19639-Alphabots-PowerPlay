/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Auto_BLUE1")
public class aaBLUE_1_AUTO extends LinearOpMode
{
    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;
    double parkingPos;


    Servo fourBarServo;
    Servo fourBarServo2;
    DcMotor rhArmMotor;
    DcMotor lhArmMotor;
    public double targetPositionHor = 0;
    public static double pos = 0.5;
    SleeveDetection.ParkingPosition snapshot = SleeveDetection.ParkingPosition.CENTER; //default

    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        rhArmMotor = hardwareMap.get(DcMotorEx.class,"rhArmMotor");
        lhArmMotor = hardwareMap.get(DcMotorEx.class, "lhArmMotor");

        rhArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lhArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rhArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lhArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rhArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lhArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rhArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        fourBarServo = hardwareMap.servo.get("fourBarServo");
        fourBarServo2 = hardwareMap.servo.get("fourBarServo2");

        fourBarServo.setDirection(Servo.Direction.REVERSE);
        fourBarServo.setPosition(0.5);
        fourBarServo2.setPosition(0.5);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        camera.setPipeline(sleeveDetection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.SIDEWAYS_RIGHT); //upright before
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        Pose2d startPose = new Pose2d(35, 60, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        Trajectory main = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(35, 34))

                .build();

        Trajectory left = drive.trajectoryBuilder(main.end())
                .strafeTo(new Vector2d(58, 34))
                .build();

        Trajectory right = drive.trajectoryBuilder(main.end())
                .strafeTo(new Vector2d(12, 34))
                .build();



        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime Analysis", sleeveDetection.getPosition());
            telemetry.update();
            sleep(150);
            /*
            Trajectory building in init
             */



        }
        snapshot = sleeveDetection.getPosition();

        telemetry.addData("Snapshot POST-START Analysis", snapshot);
        telemetry.update();


        waitForStart();
        while(opModeIsActive()){
            switch(snapshot){
                case LEFT:
                    horSlidesPID(0);
                    drive.followTrajectory(main);
                    drive.followTrajectory(left);
                    fourBarServo.setPosition(0.875);
                    fourBarServo2.setPosition(0.875);
                    sleep(2000);
                    break;
                case RIGHT:
                    horSlidesPID(0);
                    drive.followTrajectory(main);
                    drive.followTrajectory(right);
                    fourBarServo.setPosition(0.875);
                    fourBarServo2.setPosition(0.875);
                    sleep(2000);
                    break;
                case CENTER:
                    horSlidesPID(0);
                    drive.followTrajectory(main);
                    fourBarServo.setPosition(0.875);
                    fourBarServo2.setPosition(0.875);
                    sleep(2000);
                    break;
            }
            break;

        }



    }
    public void horSlidesPID(double targetPosition){
        double currentLiftPosition = rhArmMotor.getCurrentPosition();  // Get the position of the lift in ticks

        double liftError = (targetPosition-5) - currentLiftPosition;  // determine the error in ticks between our current lift position and where it should be
        double liftPower = 0;
        double liftError2 = (targetPosition-5) - lhArmMotor.getCurrentPosition();
        double liftPower2 = 0;

        if (liftError > 5 || liftError < -5){// Hysteresis band to provide damping/braking zone to reduce oscillations

            liftPower = ((liftError * 0.005)); // PID with P and D factor plus feed forward

            if (liftPower > .65){
                liftPower = .65;
            }
            if (liftPower < -.65){
                liftPower = -.65;
            }
        }
        else {

            liftPower = 0;

        } // end of if-else

        if (liftError2 > 5 || liftError2 < -5){              // Hysteresis band to provide damping/braking zone to reduce oscillations

            liftPower2 = ((liftError2 * 0.005));   		// PID with P only factor plus feed forward

            if (liftPower2 > .65){
                liftPower2 = .65;
            }
            if (liftPower2 < -.65){
                liftPower2 = -.65;
            }
        }
        else {
            liftPower2 = 0;

        }

        rhArmMotor.setPower(liftPower);
        lhArmMotor.setPower(liftPower);


    }


}