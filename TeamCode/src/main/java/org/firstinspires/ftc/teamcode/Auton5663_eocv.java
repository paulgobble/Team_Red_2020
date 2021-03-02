// Version 1.9.8
//WIP

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="Auton5663_eocv", group="Autonomous")
//@Disabled
public class Auton5663_eocv extends LinearOpMode {

    /* Declare OpMode members. */
    Robot robot   = new Robot();
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime segmentTime = new ElapsedTime();

    //Set up variables for erncoders to wheel distance.
    static final double     COUNTS_PER_MOTOR_REV    = 383.6 ;  //GoBilda Motor 28 counts per motor rev (28*13.7=383.6)
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 3.93734 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     LOW_SPEED             = 0.25;
    static final double     HIGH_SPEED              = 0.4;

    /* Set up a consitent telemetry display */
    Telemetry.Item TI_stageNo = telemetry.addData("Stage No.", "0");
    Telemetry.Item TI_stageDesc = telemetry.addData("Stage desc.", "Waiting for start");
    Telemetry.Item TI_elaspeTime = telemetry.addData("Elapsed Time", "%.3f", runtime);
    Telemetry.Item TI_segmentTime = telemetry.addData("Segment Time", "%.3f", segmentTime);
    Telemetry.Item TI_dataLine_1 = telemetry.addData("Version", "1.9.9");
    Telemetry.Item TI_dataLine_2 = telemetry.addData("-", "-");
    Telemetry.Item TI_dataLine_3 = telemetry.addData("-", "-");
    Telemetry.Item TI_dataLine_4 = telemetry.addData("-", "-");
    Telemetry.Item TI_dataLine_5 = telemetry.addData("-", "-");
    Telemetry.Item TI_dataLine_6 = telemetry.addData("-", "-");


    @Override
    public void runOpMode() {

        robot.setStreamingVideo(true);

        //robot.idTargetZone(Robot.TargetZones.X);

        telemetry.setAutoClear(false);
        TI_dataLine_2.setCaption("Video Streaming");
        TI_dataLine_2.setValue("Started");
        telemetry.update();


        robot.hMap(hardwareMap);

        //Reset all encoders to have a fresh start when the match starts.
        //Drive
        robot.FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Turn off RUN_TO_POSITION.
        //Drive
        robot.FLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.FRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.BLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.BRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //telemetry.addData("FLDrive:",robot.FLDrive.getCurrentPosition());
        TI_dataLine_3.setCaption("FLDrive");
        TI_dataLine_3.setValue(robot.FLDrive.getTargetPosition());
        //telemetry.addData("FRDrive:", robot.FRDrive.getCurrentPosition());
        TI_dataLine_4.setCaption("FRDrive");
        TI_dataLine_4.setValue(robot.FRDrive.getCurrentPosition());
        //telemetry.addData("BLDrive:", robot.BLDrive.getCurrentPosition());
        TI_dataLine_5.setCaption("BLDrive");
        TI_dataLine_5.setValue(robot.BLDrive.getCurrentPosition());
        //telemetry.addData("BRDrive:", robot.BRDrive.getCurrentPosition());
        TI_dataLine_6.setCaption("BRDrive");
        TI_dataLine_6.setValue(robot.BRDrive.getCurrentPosition());

        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        runtime.reset();

        watchAndReport(2);

        telemetry.addData("Checkpoint", "Left Watch and Report");
        telemetry.addData("Chicken Wing Position", robot.LaChickenWing.getCurrentPosition());
        telemetry.update();
        sleep(1000);
        robot.LaChickenWing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        prepChickenWing(3);

        telemetry.addData("Checkpoint", "Left Prep Chicken Wing");
        telemetry.addData("Chicken Wing Position", robot.LaChickenWing.getCurrentPosition());
        telemetry.update();
        sleep(1000);

        captureWobbleTarget(3);
        telemetry.addData("Checkpoint", "Left Capture Wobble Target99");
        telemetry.addData("Chicken Wing Position", robot.FLDrive.getCurrentPosition());
        telemetry.update();

        ShootForPowerShots(5);

        sleep(1000);

    } // end runOpMode

    /***********************
     *                     *
     *  Utility Functions  *
     *                     *
     ***********************/


    //Set up encoderDrive function.
    public void encoderDrive(double speed, double FLInches, double FRInches, double BLInches, double BRInches, double segmentTimeLimit)
    {
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
            segmentTime.reset();

            //Apply power to motors.
            robot.FLDrive.setPower(Math.abs(speed));
            robot.FRDrive.setPower(Math.abs(speed));
            robot.BLDrive.setPower(Math.abs(speed));
            robot.BRDrive.setPower(Math.abs(speed));

            //Detect whether or not the robot is running.
            while (opModeIsActive() &&
                    (segmentTime.seconds() < segmentTimeLimit) &&
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
            robot.FLDrive.setPower(0);
            robot.FRDrive.setPower(0);
            robot.BLDrive.setPower(0);
            robot.BRDrive.setPower(0);

            //Turn off RUN_TO_POSITION.
            robot.FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    /*******************************
     *                             *
     *  Autonomous Drive Segments  *
     *                             *
     *******************************/

    public void watchAndReport(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the timeout time
            segmentTime.reset();

            // allow the pipeline to scan the video
            robot.startScanning(true);

            // Telemetry
            TI_stageNo.setValue("1");
            TI_stageDesc.setValue("Watch and Report");
            TI_dataLine_1.setCaption("VideoScan");
            TI_dataLine_1.setValue("Started");
            TI_dataLine_2.setCaption("-");
            TI_dataLine_2.setValue("_");
            TI_dataLine_3.setCaption("-");
            TI_dataLine_3.setValue("-");
            TI_dataLine_4.setCaption("-");
            TI_dataLine_4.setValue("-");
            TI_dataLine_5.setCaption("-");
            TI_dataLine_5.setValue("-");
            TI_dataLine_6.setCaption("-");
            TI_dataLine_6.setValue("-");
            telemetry.update();

            while (opModeIsActive() && (segmentTime.seconds() < segmentTimeLimit)) {

                //telemetry.addData("Overall time:",  runtime.seconds());
                TI_elaspeTime.setValue("%.3f", runtime.seconds());
                //telemetry.addData("Segment time:", segmentTime.seconds());
                TI_segmentTime.setValue("%.3f", segmentTime.seconds());
                //telemetry.addData("Scan time:", robot.getScanCompleteTime());
                TI_dataLine_2.setCaption("Scan Time");
                TI_dataLine_2.setValue("%.3f", robot.getScanCompleteTime());
                //telemetry.addData("TZAV:", robot.getTargetZoneAverageValue());
                TI_dataLine_3.setCaption("TZAV");
                TI_dataLine_3.setValue(robot.getTargetZoneAverageValue());
                //telemetry.addData("Deciphered Target Zone", robot.getDecipheredTargetZone());
                TI_dataLine_4.setCaption("Target Zone");
                TI_dataLine_4.setValue(robot.getDecipheredTargetZone());
                telemetry.update();
            }

            // stop sampling video
            robot.setStreamingVideo(false);
            TI_dataLine_1.setValue("Stopped");

        }
    } // end watchAndReport

    public void prepChickenWing(double segmentTimeLimit)
    {
        if(opModeIsActive())
        {
            segmentTime.reset();

            int increaseWingPosition = 1384;
            robot.LaChickenWing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.LaChickenWing.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            int desiredWingPosition = robot.LaChickenWing.getCurrentPosition() + increaseWingPosition;

            // Telemetry
            TI_stageNo.setValue("2");
            TI_stageDesc.setValue("Prep Chicken Wing");
            TI_dataLine_1.setCaption("Increase Wing");
            TI_dataLine_1.setValue(increaseWingPosition);
            TI_dataLine_2.setCaption("-");
            TI_dataLine_2.setValue("_");
            TI_dataLine_3.setCaption("-");
            TI_dataLine_3.setValue("-");
            TI_dataLine_4.setCaption("-");
            TI_dataLine_4.setValue("-");
            TI_dataLine_5.setCaption("-");
            TI_dataLine_5.setValue("-");
            TI_dataLine_6.setCaption("-");
            TI_dataLine_6.setValue("-");
            telemetry.update();

            while(opModeIsActive() && (segmentTime.seconds() < segmentTimeLimit))
            {
                if(robot.LaChickenWing.getCurrentPosition() < desiredWingPosition)
                {
                    robot.LaChickenWing.setPower(0.35);
                    //telemetry.addData("Current Wing Position", robot.LaChickenWing.getCurrentPosition());
                    //telemetry.update();
                }
                else
                {
                    robot.LaChickenWing.setPower(0);
                    robot.FingerGrab(.2);
                }

                //telemetry.addData("Overall time:",  runtime.seconds());
                TI_elaspeTime.setValue("%.3f",runtime);
                //telemetry.addData("Segment time:", segmentTime.seconds());
                TI_segmentTime.setValue("%.3f", segmentTime);
                telemetry.update();
            }

        }
    }

    public void captureWobbleTarget(double segmentTimeLimit)
    {
        if(opModeIsActive())
        {
            segmentTime.reset();

            robot.FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //int increasedMovePosition = 20;
            //int desiredFLDrivePosition = robot.FLDrive.getCurrentPosition() + increasedMovePosition;

            // Telemetry
            TI_stageNo.setValue("3");
            TI_stageDesc.setValue("Capture Target");
            TI_dataLine_1.setCaption("-");
            TI_dataLine_1.setValue("-");
            TI_dataLine_2.setCaption("-");
            TI_dataLine_2.setValue("_");
            TI_dataLine_3.setCaption("FLDrive");
            TI_dataLine_3.setValue("-");
            TI_dataLine_4.setCaption("FRDrive");
            TI_dataLine_4.setValue("-");
            TI_dataLine_5.setCaption("BLDrive");
            TI_dataLine_5.setValue("-");
            TI_dataLine_6.setCaption("BRDrive");
            TI_dataLine_6.setValue("-");
            telemetry.update();

            encoderDrive(0.3, -30, -30, -30, -30, 0.3);

            while(opModeIsActive() && segmentTime.seconds() < segmentTimeLimit)
            {
                //THis is a while loop within a while loop.
                while (robot.FLDrive.isBusy() && robot.FRDrive.isBusy() && robot.BLDrive.isBusy() && robot.BRDrive.isBusy())
                {
                    robot.FLDrive.getCurrentPosition();
                    robot.FRDrive.getCurrentPosition();
                    robot.BLDrive.getCurrentPosition();
                    robot.BRDrive.getCurrentPosition();

                    //telemetry.addData("Overall time:",  runtime.seconds());
                    TI_elaspeTime.setValue("%.3f", runtime.seconds());
                    //telemetry.addData("Segment time:", segmentTime.seconds());
                    TI_segmentTime.setValue("%.3f", segmentTime.seconds());
                    TI_dataLine_3.setValue(robot.FLDrive.getCurrentPosition());
                    TI_dataLine_4.setValue(robot.FRDrive.getCurrentPosition());
                    TI_dataLine_5.setValue(robot.BLDrive.getCurrentPosition());
                    TI_dataLine_6.setValue(robot.BRDrive.getCurrentPosition());
                    telemetry.update();
                }
                robot.FingerGrab(1);
                int increaseWingPosition = 1384;
                int desiredWingPosition = robot.LaChickenWing.getCurrentPosition() + increaseWingPosition;

                if(robot.LaChickenWing.getCurrentPosition() < desiredWingPosition)
                {
                    robot.LaChickenWing.setPower(0.35);
                    //telemetry.addData("Current Wing Position", robot.LaChickenWing.getCurrentPosition());
                    //telemetry.update();
                }
                else
                {
                    robot.LaChickenWing.setPower(0);
                }
            }

        }
    }
    public void ShootForPowerShots(double segmentTimeLimit)
    {
        segmentTime.reset();

        if(opModeIsActive())
        {
            while(opModeIsActive() && (segmentTime.seconds() < segmentTimeLimit))
            {
                robot.Shooter(0.75);
                sleep(1000);
                robot.Intake.setPower(0.9);
            }

        }
    }

}  // end class Auton5663_eocv
