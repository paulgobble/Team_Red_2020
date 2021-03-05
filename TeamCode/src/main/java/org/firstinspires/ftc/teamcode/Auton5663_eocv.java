// Version 1.9.9

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    /* Set up a consistent telemetry display */
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

        //telemetry.setAutoClear(false);
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

        TI_dataLine_3.setCaption("FLDrive");
        TI_dataLine_3.setValue(robot.FLDrive.getTargetPosition());
        TI_dataLine_4.setCaption("FRDrive");
        TI_dataLine_4.setValue(robot.FRDrive.getCurrentPosition());
        TI_dataLine_5.setCaption("BLDrive");
        TI_dataLine_5.setValue(robot.BLDrive.getCurrentPosition());
        TI_dataLine_6.setCaption("BRDrive");
        TI_dataLine_6.setValue(robot.BRDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        runtime.reset();

        // Stage 01.1
        scanAndPrepChickenWing(5);  // was 3

        // Stage 01.2
        lungeForWobbleTarget (5);  // was .5

        //Stage 01.3
        captureWobbleTarget(5);  // was 3

        // Stage 02
        ShootForPowerShots(5); // was 3.5

        // Stage 03
        MoveToWhiteLineForDecision(5);  // was 5

        // Stage 04
        strafeTest(5); // was 3

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
                TI_dataLine_3.setCaption("FLDrive");
                TI_dataLine_3.setValue(robot.FLDrive.getTargetPosition());
                TI_dataLine_4.setCaption("FRDrive");
                TI_dataLine_4.setValue(robot.FRDrive.getCurrentPosition());
                TI_dataLine_5.setCaption("BLDrive");
                TI_dataLine_5.setValue(robot.BLDrive.getCurrentPosition());
                TI_dataLine_6.setCaption("BRDrive");
                TI_dataLine_6.setValue(robot.BRDrive.getCurrentPosition());
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
     **************************/

    // Stage 01.1
    // Specialist Segment
    public void scanAndPrepChickenWing(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the timeout time
            segmentTime.reset();

            // allow the pipeline to scan the video
            robot.startScanning(true);

            // Prepair for flight!
            int increaseWingPosition = 1500;
            robot.LaChickenWing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.LaChickenWing.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            int desiredWingPosition = robot.LaChickenWing.getCurrentPosition() + increaseWingPosition;

            // Telemetry
            TI_stageNo.setValue("1");
            TI_stageDesc.setValue("Watch and Prep");
            TI_dataLine_1.setCaption("VideoScan");
            TI_dataLine_1.setValue("Started");
            TI_dataLine_2.setCaption("Increase Wing by");
            TI_dataLine_2.setValue(increaseWingPosition);
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

                if(robot.LaChickenWing.getCurrentPosition() < desiredWingPosition)
                {
                    robot.LaChickenWing.setPower(0.35);
                }
                else
                {
                    robot.LaChickenWing.setPower(0);
                    robot.FingerGrab(.2);
                }

                // update time telemetry readout
                TI_elaspeTime.setValue("%.3f", runtime.seconds());
                TI_segmentTime.setValue("%.3f", segmentTime.seconds());
                TI_dataLine_2.setCaption("Scan Time");
                TI_dataLine_2.setValue("%.3f", robot.getScanCompleteTime());
                TI_dataLine_3.setCaption("TZAV");
                TI_dataLine_3.setValue(robot.getTargetZoneAverageValue());
                TI_dataLine_4.setCaption("Target Zone");
                TI_dataLine_4.setValue(robot.getDecipheredTargetZone());
                telemetry.update();
            }

            // stop sampling video
            robot.setStreamingVideo(false);
            TI_dataLine_1.setValue("Stopped");

        }
    } // end scanAndPrepChickenWing


    // STAGE 01.2
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
            TI_stageNo.setValue("01.2");                // edit this
            TI_stageDesc.setValue("Lunge for Target");  // edit this
            TI_dataLine_1.setCaption("-");
            TI_dataLine_1.setValue("-");
            TI_dataLine_2.setCaption("-");
            TI_dataLine_2.setValue("_");
            telemetry.update();

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end lungeForWobbleTarget


    // Stage 01.3
    // Specialist Segment
    public void captureWobbleTarget(double segmentTimeLimit)
    {
        if(opModeIsActive())
        {
            segmentTime.reset();

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

            //encoderDrive(0.3, -30, -30, -30, -30, 0.3);

            while(opModeIsActive() && segmentTime.seconds() < segmentTimeLimit)
            {

                robot.FingerGrab(1);

                int increaseWingPosition = 1384;

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

    // STAGE 02
    // Specialist Segment
    public void ShootForPowerShots(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Method Set up code goes here

            // Telemetry
            TI_stageNo.setValue("02");                // edit this
            TI_stageDesc.setValue("Power Shot!");  // edit this
            TI_dataLine_1.setCaption("-");
            TI_dataLine_1.setValue("-");
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

            // Check if its safe to run this method
            while (opModeIsActive() && (segmentTime.seconds() < segmentTimeLimit)) {

                robot.Shooter(0.9);
                sleep(1000);
                robot.Intake.setPower(0.9);

                // update time telemetry readout
                TI_elaspeTime.setValue("%.3f", runtime.seconds());
                TI_segmentTime.setValue("%.3f", segmentTime.seconds());

            }

        }
    } // end ShootForPowerShots


    // STAGE 03
    // Drive Segment
    public void MoveToWhiteLineForDecision(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .3;
            double FL_Distance = -60;
            double FR_distance = -60;
            double BL_distance = -60;
            double BR_distance = -60;

            // Telemetry
            TI_stageNo.setValue("03");                // edit this
            TI_stageDesc.setValue("Drive to line");  // edit this
            TI_dataLine_1.setCaption("-");
            TI_dataLine_1.setValue("-");
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

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end MoveToWhiteLineForDecision




    // STAGE 04
    // Drive Segment
    public void strafeTest(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .3;
            double FL_Distance = -10;
            double FR_distance = 10;
            double BL_distance = 10;
            double BR_distance = -10;

            // Telemetry
            TI_stageNo.setValue("04");                // edit this
            TI_stageDesc.setValue("Strafe Test");  // edit this
            TI_dataLine_1.setCaption("-");
            TI_dataLine_1.setValue("-");
            TI_dataLine_2.setCaption("-");
            TI_dataLine_2.setValue("_");
            telemetry.update();

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end strafeTest




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
            TI_stageNo.setValue("UTILITY");                // edit this
            TI_stageDesc.setValue("PIDF Tuner");            // edit this
            TI_dataLine_1.setCaption("-");
            TI_dataLine_1.setValue("-");
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
                TI_elaspeTime.setValue("%.3f", runtime.seconds());
                TI_segmentTime.setValue("%.3f", segmentTime.seconds());
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
            TI_stageNo.setValue("8888");                // edit this
            TI_stageDesc.setValue("Stage description");  // edit this
            TI_dataLine_1.setCaption("-");
            TI_dataLine_1.setValue("-");
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

            // Check if its safe to run this method
            while (opModeIsActive() && (segmentTime.seconds() < segmentTimeLimit)) {

                // do stuff

                // update time telemetry readout
                TI_elaspeTime.setValue("%.3f", runtime.seconds());
                TI_segmentTime.setValue("%.3f", segmentTime.seconds());
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
            TI_stageNo.setValue("9999");                // edit this
            TI_stageDesc.setValue("Stage description");  // edit this
            TI_dataLine_1.setCaption("-");
            TI_dataLine_1.setValue("-");
            TI_dataLine_2.setCaption("-");
            TI_dataLine_2.setValue("_");
            telemetry.update();

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end drive template



}  // end class Auton5663_eocv