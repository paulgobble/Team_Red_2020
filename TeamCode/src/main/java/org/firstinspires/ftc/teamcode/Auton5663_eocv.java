// Version 1.9.8
//WIP

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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


    @Override
    public void runOpMode() {

        telemetry.addData("Codebase", "v 1.9.8");

        robot.setStreamingVideo(true);

        telemetry.addData("Video Streaming", "Started");
        telemetry.update();

        robot.hMap(hardwareMap);

        //Tell the driver that the encoders are resetting.
        telemetry.addData("Status:", "Initialized v1.9.8");
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

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

        telemetry.addData("FLDrive:",robot.FLDrive.getCurrentPosition());
        telemetry.addData("FRDrive:", robot.FRDrive.getCurrentPosition());
        telemetry.addData("BLDrive:", robot.BLDrive.getCurrentPosition());
        telemetry.addData("BRDrive:", robot.BRDrive.getCurrentPosition());

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        runtime.reset();

        watchAndReport(30);

        /*
        telemetry.addData("Checkpoint", "Left Watch and Report");
        telemetry.addData("Chicken Wing Position", robot.LaChickenWing.getCurrentPosition());
        telemetry.update();
        sleep(3000);
        robot.LaChickenWing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        prepChickenWing(3);

        telemetry.addData("Checkpoint", "Left Prep Chicken Wing");
        telemetry.addData("Chicken Wing Position", robot.LaChickenWing.getCurrentPosition());
        telemetry.update();
        sleep(3000);

        captureWobbleTarget(5);
        telemetry.addData("Checkpoint", "Left Capture Wobble Target99");
        telemetry.addData("Chicken Wing Position", robot.FLDrive.getCurrentPosition());
        telemetry.update(); */

    } // end runOpMode

    /***********************
     *                     *
     *  Utility Functions  *
     *                     *
     ***********************/


    //Set up encoderDrive function.
    public void encoderDrive(double speed,
                             double FLInches, double FRInches, double BLInches, double BRInches,
                             double segmentTimeLimit) {
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

            while (opModeIsActive() && (segmentTime.seconds() < segmentTimeLimit)) {

                // Display it for the driver.
                telemetry.addData("Overall time:",  runtime.seconds());
                telemetry.addData("Segment time:", segmentTime.seconds());
                telemetry.addData("TZAV:", robot.getTargetZoneAverageValue());
                telemetry.addData("Deciphered Target Zone", robot.getDecipheredTargetZone());
                telemetry.update();
            }
            //prepChickenWing(10);
        }
    } // end watchAndReport

    public void prepChickenWing(double segmentTimeLimit)
    {
        if(opModeIsActive())
        {
            //robot.LaChickenWing.setPositionPIDFCoefficients(35);

            int increaseWingPosition = 1384;
            robot.LaChickenWing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.LaChickenWing.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            int desiredWingPosition = robot.LaChickenWing.getCurrentPosition() + increaseWingPosition;

            segmentTime.reset();

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

                telemetry.addData("Overall time:",  runtime.seconds());
                telemetry.addData("Segment time:", segmentTime.seconds());
                telemetry.update();
            }

        }
    }

    public void captureWobbleTarget(double segmentTimeLimit)
    {
        if(opModeIsActive())
        {
            int increasedMovePosition = 20;
            int desiredFLDrivePosition = robot.FLDrive.getCurrentPosition() + increasedMovePosition;

            segmentTime.reset();

            while(opModeIsActive() && segmentTime.seconds() < segmentTimeLimit)
            {
                if(robot.FLDrive.getCurrentPosition() < desiredFLDrivePosition)
                {
                    robot.FLDrive.setPower(-0.7);
                    robot.FRDrive.setPower(-0.7);
                    robot.BLDrive.setPower(-0.7);
                    robot.BRDrive.setPower(-0.7);
                }
                else
                {
                    robot.FLDrive.setPower(0);
                    robot.FRDrive.setPower(0);
                    robot.BLDrive.setPower(0);
                    robot.BRDrive.setPower(0);
                    robot.FingerGrab(1);
                }
                telemetry.addData("Overall time:",  runtime.seconds());
                telemetry.addData("Segment time:", segmentTime.seconds());
                telemetry.update();
            }
        }
    }
}  // end class Auton5663_eocv
