// Version 1.9.6
/* Copyright (c) 2017 FIRST. All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted (subject to the limitations in the disclaimer below) provided that
  * the following conditions are met:
  *
  * Redistributions of source code must retain the above copyright notice, this list
  * of conditions and the following disclaimer.
  *
  * Redistributions in binary form must reproduce the above copyright notice, this
  * list of conditions and the following disclaimer in the documentation and/or
  * other materials provided with the distribution.
  *
  * Neither the name of FIRST nor the names of its contributors may be used to endorse or
  * promote products derived from this software without specific prior written permission.
  *
  * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
  * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  */

 package org.firstinspires.ftc.teamcode;

//Import all needed materials.
 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.util.ElapsedTime;
//v.1.1

 @Autonomous(name="Auto Test", group="Auto")

 public class Auton5663_starter extends LinearOpMode {

     //Set up hardware.
     Robot         robot   = new Robot();

     //Create timekeeper.
     private ElapsedTime     runtime = new ElapsedTime();

     //Set up variables.
     static final double     COUNTS_PER_MOTOR_REV    = 383.6 ;  //GoBilda Motor 28 counts per motor rev (28*13.7=383.6)
     static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
     static final double     WHEEL_DIAMETER_INCHES   = 3.93734 ;     // For figuring circumference
     static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
             (WHEEL_DIAMETER_INCHES * 3.1415);
     static final double     LOW_SPEED             = 0.25;
     static final double     HIGH_SPEED              = 0.4;


     @Override
     public void runOpMode() {

         //Set up hardware maps.
         //robot.init(hardwareMap);
         robot.hMap(hardwareMap);

         //Set Position of Servos


         //Tell the driver that the encoders are resetting.
         telemetry.addData("Status:", "Initialized v1.9.6");
         telemetry.addData("Status", "Resetting Encoders");    //
         telemetry.update();

         // Reverse the motor that runs backwards when connected directly to the battery
         robot.FRDrive.setDirection(DcMotor.Direction.FORWARD);
         robot.FLDrive.setDirection(DcMotor.Direction.REVERSE);
         robot.BRDrive.setDirection(DcMotor.Direction.FORWARD);
         robot.BLDrive.setDirection(DcMotor.Direction.REVERSE);

         //Reset all encoders to have a fresh start when the match starts.
         //Drive
         robot.FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         robot.FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         robot.BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         robot.BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         //Turn off RUN_TO_POSITION.
         //Drive
         robot.FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         robot.FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         robot.BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         robot.BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


         //Send telemetry message to indicate successful encoder reset.
         telemetry.addData("Path0", "Starting at %7d :%7d",
                 robot.FLDrive.getCurrentPosition(),
                 robot.FRDrive.getCurrentPosition(),
                 robot.BLDrive.getCurrentPosition(),
                 robot.BRDrive.getCurrentPosition());
         telemetry.update();

         waitForStart();

         //Complete each step.

         // Step 01 - Wing Up


         //Step 1 - Drive Forward
         encoderDrive(LOW_SPEED, 60, 60, 60, 28, 60);
         sleep(500);



     }

     //Set up encoderDrive function.
     public void encoderDrive(double speed,
                              double FLInches, double FRInches, double BLInches, double BRInches,
                              double timeoutS) {
         //Create targets for motors.
         int newFLTarget;
         int newFRTarget;
         int newBLTarget;
         int newBRTarget;

         // Ensure that the opmode is still active.
         if (opModeIsActive()) {

             //Get new target positions.
             newFLTarget = robot.FLDrive.getCurrentPosition() + (int)(FLInches * COUNTS_PER_INCH);
             newFRTarget = robot.FRDrive.getCurrentPosition() + (int)(FRInches * COUNTS_PER_INCH);
             newBLTarget = robot.BLDrive.getCurrentPosition() + (int)(BLInches * COUNTS_PER_INCH);
             newBRTarget = robot.BRDrive.getCurrentPosition() + (int)(BRInches * COUNTS_PER_INCH);

             //Set the new target positions.
             robot.FLDrive.setTargetPosition(newFLTarget);
             robot.FRDrive.setTargetPosition(newFRTarget);
             robot.BLDrive.setTargetPosition(newBLTarget);
             robot.BRDrive.setTargetPosition(newBRTarget);

             //Set mode to run to position.
             robot.FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             robot.FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             robot.BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             robot.BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


             //Reset time.
             runtime.reset();

             //Apply power to motors.
             robot.FLDrive.setPower(Math.abs(speed));
             robot.FRDrive.setPower(Math.abs(speed));
             robot.BLDrive.setPower(Math.abs(speed));
             robot.BRDrive.setPower(Math.abs(speed));

             //Detect whether or not the robot is running.
             while (opModeIsActive() &&
                     (runtime.seconds() < timeoutS) &&
                     (robot.FLDrive.isBusy() && robot.FRDrive.isBusy()&&robot.BLDrive.isBusy()&&robot.BRDrive.isBusy())) {

                 //Tell driver what the robot is doing.
                 telemetry.addData("Path1",  "Running to %7d :%7d", newFLTarget,  newFRTarget, newBLTarget, newBRTarget);
                 telemetry.addData("Path2",  "Running at %7d :%7d",
                         robot.FLDrive.getCurrentPosition(),
                         robot.FRDrive.getCurrentPosition(),
                         robot.BLDrive.getCurrentPosition(),
                         robot.BRDrive.getCurrentPosition());
                 telemetry.update();
             }

             //Stop motors.
             robot.StopRobot();

             //Turn off RUN_TO_POSITION.
             robot.FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             robot.FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             robot.BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             robot.BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         }
     }
 }

