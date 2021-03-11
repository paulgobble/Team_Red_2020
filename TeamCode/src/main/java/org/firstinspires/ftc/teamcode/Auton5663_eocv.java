// Version 1.9.9

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Random;

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

    /* Set up a consistent telemetry display
    Telemetry.Item TI_stageNo = telemetry.addData("Stage No.", "0");
    Telemetry.Item TI_stageDesc = telemetry.addData("Stage desc.", "Waiting for start");
    Telemetry.Item TI_elaspeTime = telemetry.addData("Elapsed Time", "%.3f", runtime);
    Telemetry.Item TI_segmentTime = telemetry.addData("Segment Time", "%.3f", segmentTime);
    Telemetry.Item TI_dataLine_1 = telemetry.addData("Version", "1.9.9");
    Telemetry.Item TI_dataLine_2 = telemetry.addData("-", "-");
    Telemetry.Item TI_dataLine_3 = telemetry.addData("-", "-");
    Telemetry.Item TI_dataLine_4 = telemetry.addData("-", "-");
    Telemetry.Item TI_dataLine_5 = telemetry.addData("-", "-");
    Telemetry.Item TI_dataLine_6 = telemetry.addData("-", "-"); */



    @Override
    public void runOpMode() {

        robot.setStreamingVideo(true);

        //robot.idTargetZone(Robot.TargetZones.X);

        //telemetry.setAutoClear(false);
        telemetry.addData("Video Streaming", "Started");
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

        telemetry.addData("FLDrive", robot.FLDrive.getTargetPosition());
        telemetry.addData("FRDrive", robot.FRDrive.getTargetPosition());
        telemetry.addData("BLDrive", robot.BLDrive.getTargetPosition());
        telemetry.addData("BRDrive", robot.BRDrive.getTargetPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        // THE SCRIPT (aka calling the functions)
        // These are all stages / segments / methods
        // that are executed regardless of how many rings are places,
        // common actions taken before the Postscripts diverge

        // Stage 01.1
        scanAndPrepChickenWing(3);

        // Stage 01.2
        lungeForWobbleTarget(5);  // was .5

        //Stage 01.3
        captureWobbleTarget(3);  // was 3

        // Stage 02
        ShootForPowerShots(2.5); // was 3.5

        // Stage 03.1
        MoveToWhiteLineForDecision(3);  // was 5

        // Stage 03.2
        carefullyDriveAtopLine(2);


        // THE POSTSCRIPT (aka calling he functions unique
        // to the the there possibel postscripts: A, B, or C

        switch (robot.getDecipheredTargetZone()) {
        case A:
            telemetry.addData("Postscript", "A");
            // Stage 10_A  (all Postscript stages start at 10)
            pivot_A(5);

            // Postscript Stage 11_A
            driveToZone_A(3);

            // Postscript Stage 12_A
            pivotForZoneA(3);

            driveToZone_A2(3);

            //moveBackForDroppingTarget(3);
            // Postscript Stage 13_A
            lowerChickenWing(3);

            // Postscript Stage 14_A
            dropWobbleTarget(2);

            // Postscropt Stage 15_A
            raiseChickenWing(3);

            //moveToWhiteLineForZoneA(3);
        break;

        case B:
            telemetry.addData("Postscript chose", "B");
            // Stage 10_B
            pivot_B_C(5);

            // Postscript Stage 11_B
            driveToZone_B(3);

            // Postscript Stage 12_B
            lowerChickenWing(3);

            // Postscript Stage 13_B
            dropWobbleTarget(2);

            // Postscropt Stage 14_B
            raiseChickenWing(3);

            // Postscript Stage 14_B
            strafeFor_B_ToWhiteLine(2);

            // Stage 11_B
            // more B stuff
        break;

        case C:
            telemetry.addData("Postscript chosen", "C");
            // Stage 10_C
            pivot_B_C(5);

            // Postscript Stage 11_C
            driveToZone_C(3);

            // Stage 12_C
            rotateForC(3);

            // Stage 13_C
            driveToZoneC(3);

            // Stage 14_C
            pivotForZoneC(3);

            // Stage 15_C
            lowerChickenWing(3);

            // Stage 16_C
            dropWobbleTarget(2);

            // Stage 17_C
            raiseChickenWing(3);

            // Stage 18_C
            moveToWhiteLineForZoneC(4);

            //pivot_180(3);

            break;

        default:
            telemetry.addData("Postscript chose", "None");
            // flamethrowers!
            // we probably wont need this final default case
    }
        // UTILITY
        //pidfTuner_utility(4);
    } // end runOpMode




    /***********************
     *                     *
     *   Drive Functions   *
     *                     *
     ***********************/

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

                telemetry.addData("FLDrive", robot.FLDrive.getCurrentPosition());
                telemetry.addData("FRDrive", robot.FRDrive.getCurrentPosition());
                telemetry.addData("BLDrive", robot.BLDrive.getCurrentPosition());
                telemetry.addData("BRDrive", robot.BRDrive.getCurrentPosition());
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
    } //encoderDrive




    /**************************
     *                        *
     *  Autonomous  Segments  *
     *                        *
     *        SCRIPT          *
     *                        *
     **************************/

    // SCRIPT Stage 01.1
    // Specialist Segment
    public void scanAndPrepChickenWing(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the timeout time
            segmentTime.reset();

            // allow the pipeline to scan the video
            robot.startScanning(true);

            // Prepair for flight!
            int increaseWingPosition = 1251;
            robot.LaChickenWing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.LaChickenWing.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            int desiredWingPosition = robot.LaChickenWing.getCurrentPosition() + increaseWingPosition;

            // Telemetry
            telemetry.addData("Stage No", "01.1");
            telemetry.addData("Stage Desc","Scan and Prep");
            telemetry.addData("VideoScan", "Started");
            telemetry.update();

            while (opModeIsActive() && (segmentTime.seconds() < segmentTimeLimit)) {

                if(robot.LaChickenWing.getCurrentPosition() < desiredWingPosition)
                {
                    robot.LaChickenWing.setPower(0.5);
                }
                else
                {
                    robot.LaChickenWing.setPower(0);
                    robot.FingerGrab(.6);
                }

                // update time telemetry readout
                telemetry.addData("Runtime", "%.3f", runtime.seconds());
                telemetry.addData("Segment time", "%.3f", segmentTime.seconds());
                telemetry.addData("Final Scan time", "%.3f", robot.getScanCompleteTime());
                telemetry.addData("TZAV", robot.getTargetZoneAverageValue());
                telemetry.addData("Target Zone", robot.getDecipheredTargetZone());
                telemetry.update();
            }

            // stop sampling video
            robot.setStreamingVideo(false);
            telemetry.addData("VideoScan", "Stopped");
            telemetry.update();

        }
    } // end scanAndPrepChickenWing


    // SCRIPT STAGE 01.2
    // Drive Segment
    public void lungeForWobbleTarget(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .2;
            double FL_Distance = -5;
            double FR_distance = -5;
            double BL_distance = -5;
            double BR_distance = -5;

            // Telemetry
            telemetry.addData("Stage No", "01.2");
            telemetry.addData("Stage Desc", "Lunge for Wobble");
            telemetry.update();

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end lungeForWobbleTarget


    // SCRIPT Stage 01.3
    // Specialist Segment
    public void captureWobbleTarget(double segmentTimeLimit)
    {
        if(opModeIsActive())
        {
            segmentTime.reset();

            // Telemetry
            telemetry.addData("Stage No", "01.3");
            telemetry.addData("Stage Desc", "Capture Wobble");
            telemetry.update();

            while(opModeIsActive() && segmentTime.seconds() < segmentTimeLimit)
            {
                robot.FingerGrab(1);

                sleep(1000);

                int increaseWingPosition = 500;

                int desiredWingPosition = robot.LaChickenWing.getCurrentPosition() + increaseWingPosition;

                if(robot.LaChickenWing.getCurrentPosition() < desiredWingPosition)
                {
                   robot.LaChickenWing.setPower(0.35);
                }
                else
                {
                   robot.LaChickenWing.setPower(0);
                }
            }

        }
    }

    // SCRIPT STAGE 02
    // Specialist Segment
    public void ShootForPowerShots(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            boolean canFire = true;

            // Method Set up code goes here
            double speed = .275;
            double FL_Distance1 = 7.5;
            double FR_distance1 = -7.5;
            double BL_distance1 = 7.5;
            double BR_distance1 = -7.5;

            double FL_Distance2 = -4;
            double FR_distance2 = 4;
            double BL_distance2 = -4;
            double BR_distance2 = 4;

            // Telemetry
            telemetry.addData("Stage No", "02");
            telemetry.addData("Stage Desc", "Shoot for Power");
            telemetry.update();

            // Check if its safe to run this method
            while (opModeIsActive() && (segmentTime.seconds() < segmentTimeLimit) && canFire ) {

                robot.Shooter(0.85);
                sleep(1000);
                robot.Intake.setPower(0.9);
                encoderDrive(speed, FL_Distance2, FR_distance2, BL_distance2, BR_distance2, segmentTimeLimit);
                sleep(1000);
                canFire = false;
                // update time telemetry readout
                telemetry.addData("Runtime", "%.3f", runtime.seconds());
                telemetry.addData("Segment time", "%.3f", segmentTime.seconds());
                telemetry.update();

            }
            robot.Shooter(0);
            robot.Intake.setPower(0);
        }
    } // end ShootForPowerShots

    // SCRIPT STAGE 03.1
    // Drive Segment
    // Drive quickly and stop just shy of White line
    public void MoveToWhiteLineForDecision(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .65;
            double FL_Distance = -41;
            double FR_distance = -41;
            double BL_distance = -41;
            double BR_distance = -41;

            // Telemetry
            telemetry.addData("Stage No", "03.1");
            telemetry.addData("Stage Desc", "Move to White Line");
            telemetry.update();

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end MoveToWhiteLineForDecision



    // SCRIPT STAGE 3.2
    // Specialist Segment
    // Drive slowly until the front right color sensor see the whiteline
    public void carefullyDriveAtopLine(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Method Set up code goes here

            // Telemetry
            telemetry.addData("Stage No", "03.2");
            telemetry.addData("Stage Desc", "Drive Carefully atop line");
            telemetry.update();

            // Check if its safe to run this method
            while (opModeIsActive() && (segmentTime.seconds() < segmentTimeLimit)) {

                // do stuff
                while (robot.getFRColor_alpha() < 20) {

                    robot.MecanumDrive(0.2, 0, 0);

                }

                // update time telemetry readout
                telemetry.addData("Runtime", "%.3f", runtime.seconds());
                telemetry.addData("Segment time", "%.3f", segmentTime.seconds());
                telemetry.update();
            }

        }
    } // end specialist template




    /**************************
     *                        *
     *  Autonomous  Segments  *
     *                        *
     *    POSTSCRIPT A        *
     *                        *
     **************************/


    // POSTSCRIPT STAGE 10_A
    // Drive Segment
    public void pivot_A(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .3;
            double FL_Distance = 21;
            double FR_distance = -21;
            double BL_distance = 21;
            double BR_distance = -21;

            // Telemetry
            telemetry.addData("Stage No", "10 A");
            telemetry.addData("Stage Desc", "Pivot to Box A");
            telemetry.update();

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end pivot_A

    public void pivot_180(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .3;
            double FL_Distance = 90;
            double FR_distance = -90;
            double BL_distance = 90;
            double BR_distance = -90;

            // Telemetry
            telemetry.addData("Stage No", "10 A");
            telemetry.addData("Stage Desc", "Pivot to Box A");
            telemetry.update();

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end pivot_180


    // STAGE 11 A
    // Drive Segment
    public void driveToZone_A(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .3;
            double FL_Distance = -25; // was 20
            double FR_distance = -25;
            double BL_distance = -25;
            double BR_distance = -25;

            // Telemetry
            telemetry.addData("Stage No", "11 A");
            telemetry.addData("Stage Desc", "Drive to A zone");
            telemetry.update();

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end driveToZone_A

    public void driveToZone_A2(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .3;
            double FL_Distance = -12.5; // was 20
            double FR_distance = -12.5;
            double BL_distance = -12.5;
            double BR_distance = -12.5;

            // Telemetry
            telemetry.addData("Stage No", "11 A");
            telemetry.addData("Stage Desc", "Drive to A zone");
            telemetry.update();

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end driveToZone_A2

    // Postscript Stage 12_A
    // Drive Segment
    public void pivotForZoneA(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .5;
            double FL_Distance = 22;
            double FR_distance = -22;
            double BL_distance = 22;
            double BR_distance = -22;

            // Telemetry
            telemetry.addData("Stage No", "12_A");
            telemetry.addData("Stage Desc", "pivotForZoneA");
            telemetry.update();

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end pivotForZoneA

    // Postscript Stage 13_A (and several other post script locations)
    // Specialist Segment
    public void lowerChickenWing(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Method Set up code goes here

            // Telemetry
            telemetry.addData("Stage No", "13_Z");
            telemetry.addData("Stage Desc", "lower La Chicken Wing");
            telemetry.update();

            // Check if its safe to run this method
            while (opModeIsActive() && (segmentTime.seconds() < segmentTimeLimit)) {

                int increaseWingPosition = 1421;

                //int desiredWingPosition = robot.LaChickenWing.getCurrentPosition() + increaseWingPosition;

                if(robot.LaChickenWing.getCurrentPosition() > increaseWingPosition)
                {
                    robot.LaChickenWing.setPower(-0.35);
                }
                else
                {
                    robot.LaChickenWing.setPower(0);
                }

                // update time telemetry readout
                telemetry.addData("Runtime", "%.3f", runtime.seconds());
                telemetry.addData("Segment time", "%.3f", segmentTime.seconds());
                telemetry.update();
            }

        }
    } // end lowerChickenWing


    // Postscript Stage 14_A
    // Specialsit Segment
    public void dropWobbleTarget(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Method Set up code goes here

            // Telemetry
            telemetry.addData("Stage No", "14_A");
            telemetry.addData("Stage Desc", "Drop Wobble Target");
            telemetry.update();

            // Check if its safe to run this method
            while (opModeIsActive() && (segmentTime.seconds() < segmentTimeLimit)) {

                // Drop target

                robot.FingerGrab(.6);

                // update time telemetry readout
                telemetry.addData("Runtime", "%.3f", runtime.seconds());
                telemetry.addData("Segment time", "%.3f", segmentTime.seconds());
                telemetry.update();
            }

        }
    } //End drop wobble target.


    /**************************
     *                        *
     *  Autonomous  Segments  *
     *                        *
     *    POSTSCRIPT B        *
     *                        *
     **************************/

    // POSTSCRIPT STAGE 10_B_C
    // Drive Segment
    public void pivot_B_C(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .3;
            double FL_Distance = 17.5;
            double FR_distance = -17.5;
            double BL_distance = 17.5;
            double BR_distance = -17.5;

            // Telemetry
            telemetry.addData("Stage No", "10 B C");
            telemetry.addData("Stage Desc", "Pivot to Box B or C");
            telemetry.update();

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end pivot_B


    // STAGE 11 B
    // Drive Segment
    public void driveToZone_B(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .3;
            double FL_Distance = -20;
            double FR_distance = -20;
            double BL_distance = -20;
            double BR_distance = -20;

            // Telemetry
            telemetry.addData("Stage No", "11 B");
            telemetry.addData("Stage Desc", "Drive to B zone");
            telemetry.update();

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end driveToZone_B


    // Postscript Stage 14_B
    // Drive segment
    public void strafeFor_B_ToWhiteLine (double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .5;
            double FL_Distance = 15;
            double FR_distance = -15;
            double BL_distance = -15;
            double BR_distance = 15;

            // Telemetry
            telemetry.addData("Stage No", "14_B");
            telemetry.addData("Stage Desc", "Strafe to White Line");
            telemetry.update();

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end moveBackForDroppingTarget



    /**************************
     *                        *
     *  Autonomous  Segments  *
     *                        *
     *    POSTSCRIPT C        *
     *                        *
     **************************/

    // POSTSCRIPT STAGE 10_B_C
    // Drive Segment
    // see Postscript B

    // STAGE 11 C
    // Drive Segment
    public void driveToZone_C(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .3;
            double FL_Distance = -30;
            double FR_distance = -30;
            double BL_distance = -30;
            double BR_distance = -30;

            // Telemetry
            telemetry.addData("Stage No", "11 C");
            telemetry.addData("Stage Desc", "Drive to C zone");
            telemetry.update();

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end driveToZone_C


    // Postscript 14_C
    // Drive Segment
    public void pivotForZoneC(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .5;
            double FL_Distance = 47.5;
            double FR_distance = -47.5;
            double BL_distance = 47.5;
            double BR_distance = -47.5;

            // Telemetry
            telemetry.addData("Stage No", "14_C");
            telemetry.addData("Stage Desc", "Pivot for Zone C");
            telemetry.update();

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end pivotForZoneC



    // Postscript Stage 18_C
    public void moveToWhiteLineForZoneC(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .5;
            double FL_Distance = -29;
            double FR_distance = -29;
            double BL_distance = -29;
            double BR_distance = -29;

            // Telemetry
            telemetry.addData("Stage No", "18_C");
            telemetry.addData("Stage Desc", "Drive to White Line");
            telemetry.update();

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);
        }
    } // end moveToWhiteLineForZoneA



    public void raiseChickenWing(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Method Set up code goes here

            // Telemetry
            telemetry.addData("Stage No", "888");
            telemetry.addData("Stage Desc", "sepecist stuff");
            telemetry.update();

            // Check if its safe to run this method
            while (opModeIsActive() && (segmentTime.seconds() < segmentTimeLimit)) {

                int increaseWingPosition = 750;

                int desiredWingPosition = robot.LaChickenWing.getCurrentPosition() + increaseWingPosition;

                if(robot.LaChickenWing.getCurrentPosition() < desiredWingPosition)
                {
                    robot.LaChickenWing.setPower(0.35);
                }
                else
                {
                    robot.LaChickenWing.setPower(0);
                }


                // update time telemetry readout
                telemetry.addData("Runtime", "%.3f", runtime.seconds());
                telemetry.addData("Segment time", "%.3f", segmentTime.seconds());
                telemetry.update();
            }

        }
    } // end lowerChickenWing

    public void rotateForC(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .5;
            double FL_Distance = -20;
            double FR_distance = 20;
            double BL_distance = -20;
            double BR_distance = 20;

            // Telemetry
            telemetry.addData("Stage No", "12_C");
            telemetry.addData("Stage Desc", "rotateForC");
            telemetry.update();

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);
        }
    } // end rotateForC

    public void driveToZoneC(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .3; // was .5
            double FL_Distance = -35; // was 30
            double FR_distance = -35;
            double BL_distance = -35;
            double BR_distance = -35;

            // Telemetry
            telemetry.addData("Stage No", "13_C");
            telemetry.addData("Stage Desc", "driveToZoneC");
            telemetry.update();

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end driveToZoneC

    /**************************
     *                        *
     *  OFF SCRIPT UTILITIES  *
     *                        *
     **************************/

    // Testing utlity for PIDF
    // Specialist Segment
    public void pidfTuner_utility(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Method Set up code goes here
            robot.FLDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            robot.FRDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            robot.BLDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            robot.BRDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


            double currentVelocity;
            double maxVelocity = 0.0;

            double P;
            double I;
            double D;
            double F;

            // Telemetry
            telemetry.addData("Stage No", "UTILITY");
            telemetry.addData("Stage Desc", "PIDF Tuner");
            telemetry.update();

            // Check if its safe to run this method
            while (opModeIsActive() && (segmentTime.seconds() < segmentTimeLimit)) {

                currentVelocity = robot.BRDrive.getVelocity();

                robot.FLDrive.setPower(1);
                robot.FRDrive.setPower(1);
                robot.BLDrive.setPower(1);
                robot.BRDrive.setPower(1);

                if (currentVelocity > maxVelocity) {
                    maxVelocity = currentVelocity;
                }

                telemetry.addData("Motor", "BRDrive");
                //telemetry.addData("current velocity", currentVelocity);
                telemetry.addData("maximum velocity", maxVelocity);
                telemetry.update();

                F = 32767 / maxVelocity;
                P = 0.1 * F;
                I = 0.1 * P;
                D = 0;

                //telemetry.addData("P",  P);
                //telemetry.addData("I", I );
                //telemetry.addData("D", D);
                telemetry.addData("F", F);

                // update time telemetry readout
                telemetry.addData("Runtime", "%.3f", runtime.seconds());
                telemetry.addData("Segment time", "%.3f", segmentTime.seconds());
                telemetry.update();
            }

            robot.FLDrive.setPower(0);
            robot.FRDrive.setPower(0);
            robot.BLDrive.setPower(0);
            robot.BRDrive.setPower(0);

        }
    } // end pidfTuner_utility







    /*******************************
     *                             *
     *      Specialist Segment     *
     *      Method Template        *
     *                             *
     *******************************/
    // STAGE 8888
    // Specialist Segment
    public void specialist_segment_template(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Method Set up code goes here

            // Telemetry
            telemetry.addData("Stage No", "888");
            telemetry.addData("Stage Desc", "sepecist stuff");
            telemetry.update();

            // Check if its safe to run this method
            while (opModeIsActive() && (segmentTime.seconds() < segmentTimeLimit)) {

                // do stuff

                // update time telemetry readout
                telemetry.addData("Runtime", "%.3f", runtime.seconds());
                telemetry.addData("Segment time", "%.3f", segmentTime.seconds());
                telemetry.update();
            }

        }
    } // end specialist template

    /*******************************
     *                             *
     *      Drive Segment          *
     *      Method Wrapper         *
     *      Template               *
     *                             *
     *******************************/
    // STAGE 9999
    // Drive Segment
    public void drive_segment_template(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .5;
            double FL_Distance = 5;
            double FR_distance = 5;
            double BL_distance = 5;
            double BR_distance = 5;

            // Telemetry
            telemetry.addData("Stage No", "999");
            telemetry.addData("Stage Desc", "Drive stuff");
            telemetry.update();

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end drive template
}  // end class Auton5663_eocv