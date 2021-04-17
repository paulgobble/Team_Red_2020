// Version 2.0

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auton5663_eocv", group="Autonomous")
//@Disabled
public class Auton5663_eocv extends LinearOpMode {

    /* Declare OpMode members. */
    Robot robot = new Robot();

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime segmentTime = new ElapsedTime();

    //Set up variables for erncoders to wheel distance.
    static final double     COUNTS_PER_MOTOR_REV    = 383.6 ;  //GoBilda Motor 28 counts per motor rev (28*13.7=383.6)
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 3.93734 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    /* Store data for telemetry */
    private String stageNo = "0";
    private String stageDescription = "Awake";
    private String flexLine_1_caption = "-";
    private String flexLine_1_value = "-";
    private String flexLine_2_caption = "-";
    private String flexLine_2_value = "-";
    private String flexLine_3_caption = "-";
    private String flexLine_3_value = "-";
    private final String codeVersion = "2.0 Novi + Color Sensors";

    /* Define enum for possible modes in the explainYoursefl unified telemetry method  */
    enum mode {
        Transmit,   // send the telemetry data
        Reset       // reset the data
    };


    @Override
    public void runOpMode() {

        robot.setStreamingVideo(true);

        //robot.idTargetZone(Robot.TargetZones.X);

        robot.hMap(hardwareMap);

        //Reset all encoders to have a fresh start when the match starts.
        //Drive
        //robot.FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Turn off RUN_TO_POSITION.
        //Drive
        robot.FLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.FRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.BLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.BRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Telemetry
        explainYourself(mode.Transmit);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        // THE SCRIPT (aka calling the functions)
        // These are all stages / segments / methods
        // that are executed regardless of how many rings are places,
        // common actions taken before the Postscripts diverge

        // Stage 01.1
        scanAndPrepChickenWing(2.75);

        // Stage 01.2
        lungeForWobbleTarget(2.75);  // was .5

        //Stage 01.3
        captureWobbleTarget(2.75);  // was 3

        // Stage 01.4
        MoveToWhiteLineToShoot(5);  // was 5

        // Stage 02 (option 1)
        //ShootForPowerShots(3.5); // was 3.5

        // Stage 02 (option 2)
        strafe_n_shoot(5);

        // Stage 03.2
        carefullyDriveAtopLine(3); // was 1

        //StrafeBack(3);  // was 5


        // THE POSTSCRIPT (aka calling he functions unique
        // to the the there possibel postscripts: A, B, or C

        switch (robot.getDecipheredTargetZone()) {
        case A:
            //telemetry.addData("Postscript", "A");
            // Stage 10_A  (all Postscript stages start at 10)
            pivot_A(5);

            // Postscript Stage 11_A
            driveToZone_A(4);

            // Postscript Stage 11.5_A
            driveToZone_A_stage_2(4);

            // Postscript Stage 12_A
            pivotForZoneA(4);

            // Postscript Stage 13_A
            backIntoZoneA(4);

            // Postscript Stage 20_Z
            lowerChickenWing(3);

            // Postscript Stage 21_Z
            releaseWobbleTarget(1);

            // Postscropt Stage 22_Z
            raiseChickenWing(3);

        break;

        case B:
            //telemetry.addData("Postscript chose", "B");
            // Stage 10_B
            pivot_B_C(3);

            // Postscript Stage 11_B
            driveToZone_B(3);

            // Postscript Stage 20_Z
            lowerChickenWing(3);

            // Postscript Stage 21_Z
            releaseWobbleTarget(2);

            // Postscropt Stage 22_Z
            raiseChickenWing(3);

            // Postscript Stage 23_B
            strafeFor_B_ToWhiteLine(2);

        break;

        case C:
            //telemetry.addData("Postscript chosen", "C");

            //Postscript Stage 1_C
            driveNearWall(4);

            //Postscript Stage 2_C
            driveCloseToWall(5); // was 3

            //Postscript Stage 3_C
            turnToStrafeC(4);

            //Postscript Stage 4_C
            backIntoWall(4);

            //Postscript Stage 5_C
            rollForward(3);

            //Postscript Stage 6_C
            strafeForC(4);

            //Postscript Stage 7_C
            backIntoWall(4);

            //Postcript Stage 8_C
            rollForward(3);



            break;

        default:
            telemetry.addData("Postscript chose", "None");
            // flamethrowers!
            // we probably wont need this final default case
    }
        // UTILITY
        //pidfTuner_utility(4);
    } // end runOpMode




    /*********************************
     *                               *
     *   Internal opMode Functions   *
     *                               *
     *********************************/



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

                //Telemetry
                explainYourself(mode.Transmit);

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

    // unified Telemetry management
    public void explainYourself(mode thisMode){

        if (thisMode == mode.Transmit) {

            telemetry.addLine("Runtime | ")
                    .addData("TOTAL", "%.3f", runtime.seconds())
                    .addData("SEGMENT", "%.3f", segmentTime.seconds());

            telemetry.addLine("Stage | ")
                    .addData("No.", stageNo)
                    .addData("-", stageDescription);

            telemetry.addData(flexLine_1_caption, flexLine_1_value);
            telemetry.addData(flexLine_2_caption, flexLine_2_value);
            telemetry.addData(flexLine_3_caption, flexLine_3_value);

            if (robot.FRDrive.getMode() == DcMotorEx.RunMode.RUN_TO_POSITION) {

                telemetry.addLine("FLDrive | ")
                        .addData("Current", "|%05d|", robot.FLDrive.getCurrentPosition())
                        .addData("Target", "|%05d|", robot.FLDrive.getTargetPosition());
                telemetry.addLine("FRDrive | ")
                        .addData("Current", "|%05d|", robot.FRDrive.getCurrentPosition())
                        .addData("Target", "|%05d|", robot.FRDrive.getTargetPosition());
                telemetry.addLine("BLDrive | ")
                        .addData("Current", "|%05d|", robot.BLDrive.getCurrentPosition())
                        .addData("Target", "|%05d|", robot.BLDrive.getTargetPosition());
                telemetry.addLine("BRDrive | ")
                        .addData("Current", "|%05d|", robot.BRDrive.getCurrentPosition())
                        .addData("Target", "|%05d|", robot.BRDrive.getTargetPosition());
            } else {

                telemetry.addLine("FLDrive | ")
                        .addData("Current", "|%05d|", robot.FLDrive.getCurrentPosition())
                        .addData("Target", "none");
                telemetry.addLine("FRDrive | ")
                        .addData("Current", "|%05d|", robot.FRDrive.getCurrentPosition())
                        .addData("Target", "none");
                telemetry.addLine("BLDrive | ")
                        .addData("Current", "|%05d|", robot.BLDrive.getCurrentPosition())
                        .addData("Target", "none");
                telemetry.addLine("BRDrive | ")
                        .addData("Current", "|%05d|", robot.BRDrive.getCurrentPosition())
                        .addData("Target", "none");
            }

            telemetry.addData("Code Version", codeVersion);

            telemetry.update();

        } else if (thisMode == mode.Reset) {

            flexLine_1_caption = "-";
            flexLine_1_value = "-";
            flexLine_2_caption = "-";
            flexLine_2_value = "-";
            flexLine_3_caption = "-";
            flexLine_3_value = "-";

        }

    } // end explainYourself




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
            explainYourself(mode.Reset);
            stageNo = "01.1";  //telemetry.addData("Stage No", "01.1");
            stageDescription = "Scan and Prep";  //telemetry.addData("Stage Desc","Scan and Prep");
            flexLine_1_caption = "VideoScan"; //telemetry.addData("VideoScan", "Started");
            flexLine_1_value = "Started";
            explainYourself(mode.Transmit); //telemetry.update();

            while (opModeIsActive() && (segmentTime.seconds() < segmentTimeLimit)) {

                if(robot.LaChickenWing.getCurrentPosition() < desiredWingPosition)
                {
                    robot.LaChickenWing.setPower(0.5);
                }
                else
                {
                    robot.LaChickenWing.setPower(0);
                    robot.FingerGrab(.6);
                    sleep(500);
                    break;
                }

                flexLine_2_caption = "TZAV"; //telemetry.addData("TZAV", robot.getTargetZoneAverageValue());
                flexLine_2_value = String.valueOf(robot.getTargetZoneAverageValue());
                flexLine_3_caption = "Target Zone"; //telemetry.addData("Target Zone", robot.getDecipheredTargetZone());
                flexLine_3_value = String.valueOf(robot.getDecipheredTargetZone());
                explainYourself(mode.Transmit); // telemetry.update();
            }

            // stop sampling video
            robot.setStreamingVideo(false);
            flexLine_1_value = "stopped"; // telemetry.addData("VideoScan", "Stopped");
            explainYourself(mode.Transmit); //telemetry.update();

        }
    } // end scanAndPrepChickenWing

    // SCRIPT STAGE 01.2
    // Drive Segment
    public void lungeForWobbleTarget(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Telemetry
            explainYourself(mode.Reset);
            stageNo = "01.2";
            stageDescription = "Lunge for Wobble Target";
            explainYourself(mode.Transmit);

            // Drive Targets
            double speed = .2;
            double FL_Distance = -5;
            double FR_distance = -5;
            double BL_distance = -5;
            double BR_distance = -5;

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end lungeForWobbleTarget


    // SCRIPT Stage 01.3
    // Specialist Segment
    public void StrafeBack(double segmentTimeLimit)
    {
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .65;
            double FL_Distance = -4; // was -41, then exparimented with carefullyDriveAtopLine
            double FR_distance = 4;
            double BL_distance = 4;
            double BR_distance = -4;


            // Telemetry
            explainYourself(mode.Reset);
            stageNo = "03.1";
            stageDescription = "Race up to White Line";
            explainYourself(mode.Transmit);

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    }




    // SCRIPT Stage 01.3
    // Specialist Segment
    public void captureWobbleTarget(double segmentTimeLimit)
    {
        if(opModeIsActive())
        {
            segmentTime.reset();

            // Telemetry
            explainYourself(mode.Reset);
            stageNo = "01.3";
            stageDescription = "Capture Wobble Target";
            explainYourself(mode.Transmit);

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
                   break; // try a break statement here
                }
            }

        }
    }



    // SCRIPT STAGE 01.4
    // Drive Segment
    // Drive quickly and stop just shy of White line
    public void MoveToWhiteLineToShoot(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .4;
            double FL_Distance = -41; // was -41, then exparimented with carefullyDriveAtopLine
            double FR_distance = -41;
            double BL_distance = -41;
            double BR_distance = -41;


            // Telemetry
            explainYourself(mode.Reset);
            stageNo = "03.1";
            stageDescription = "Race up to White Line";
            explainYourself(mode.Transmit);

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end MoveToWhiteLineToShoot



    // SCRIPT STAGE 02
    // Specialist Segment
    public void ShootForPowerShots(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Method Set up code goes here
            boolean canFire = true;

            // Telemetry
            explainYourself(mode.Reset);
            stageNo = "02";
            stageDescription = "Sweep Power Shot";
            explainYourself(mode.Transmit);

            robot.Shooter(.75);
            sleep(1500);


            // Check if its safe to run this method
            while (opModeIsActive() && (segmentTime.seconds() < segmentTimeLimit)) {

                robot.MecanumDrive(0, 0.375,0);


                if(canFire)
                {
                    robot.Intake.setPower(.8);
                    sleep(1000);
                    canFire = false;
                }
                else
                {
                    sleep(500);
                    canFire = true;
                }

                //encoderDrive(speed, -Drive_Distance1, Drive_Distance1, -Drive_Distance1, Drive_Distance1, 0.4);
                //encoderDrive(speed, Drive_Distance2, -Drive_Distance2, Drive_Distance2, -Drive_Distance2, 0.4);
                // update time telemetry readout
                //telemetry.addData("Runtime", "%.3f", runtime.seconds());
                //telemetry.addData("Segment time", "%.3f", segmentTime.seconds());
                explainYourself(mode.Transmit); //telemetry.update();

            }
            robot.Shooter(0);
            robot.Intake.setPower(0);
        }
    } // end ShootForPowerShots


    // STAGE 02 (option 2)
    // Specialist Segment
    // Strafe using encoder counts while shooting for Power Shots
    public void strafe_n_shoot(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Method Set up code goes here
            int strafeThisFar = 275; // was 300

            robot.FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.FLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.FRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.BLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.BRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            int desiredStrafePosition = robot.FRDrive.getCurrentPosition() + strafeThisFar;  // basing strafe measurment of of just the front right drive


            // Telemetry
            explainYourself(mode.Reset);
            stageNo = "02";
            stageDescription = "Strafe n Shoot";
            explainYourself(mode.Transmit);

            // get the flywheels up to speed
            robot.Shooter(.8);
            sleep(1500);

            // Check if its safe to run this method
            while (opModeIsActive() && (segmentTime.seconds() < segmentTimeLimit)) {

                // do stuff
                robot.Intake.setPower(.8);

                if(robot.FRDrive.getCurrentPosition() < desiredStrafePosition) // testing just the front right drive
                {
                    robot.FLDrive.setPower(-0.4);
                    robot.FRDrive.setPower(0.4);
                    robot.BLDrive.setPower(0.4);
                    robot.BRDrive.setPower(-0.4);
                }
                else
                {
                    robot.FLDrive.setPower(0);
                    robot.FRDrive.setPower(0);
                    robot.BLDrive.setPower(0);
                    robot.BRDrive.setPower(0);

                    robot.Intake.setPower(0);
                    robot.Shooter(0);
                    break;
                }

                // update time telemetry readout
                explainYourself(mode.Transmit);
            }

        }
    } // end specialist template



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
            explainYourself(mode.Reset);
            stageNo = "03.2";
            stageDescription = "Carefully Drive Atop White Line";
            flexLine_1_caption = "Color, Alpha";
            explainYourself(mode.Transmit);

            // Check if its safe to run this method
            while (opModeIsActive() && (segmentTime.seconds() < segmentTimeLimit)) {

                if (robot.getFRColor_alpha() < 400) {

                    robot.lightBar(Robot.TargetZones.C);
                    robot.MecanumDrive(-0.3, 0, 0);

                } else {

                    robot.lightBar(Robot.TargetZones.X);
                    robot.MecanumDrive(0, 0, 0);
                    break; // untested

                }

                // Telemetry
                flexLine_1_value = String.valueOf(robot.getFRColor_alpha());
                explainYourself(mode.Transmit);
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
    // Pivot to Target Zone A
    public void pivot_A(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .2; // was .2
            double FL_Distance = 23;
            double FR_distance = -23;
            double BL_distance = 23;
            double BR_distance = -23;

            // Telemetry
            explainYourself(mode.Reset);
            stageNo = "10 A";
            stageDescription = "Pivot to Zone A";
            explainYourself(mode.Transmit);

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end pivot_A


    // STAGE 11 A
    // Drive Segment
    // Drive up to Zone A
    public void driveToZone_A(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .3;
            double FL_Distance = -27; // was -25
            double FR_distance = -27;
            double BL_distance = -27;
            double BR_distance = -27;

            // Telemetry
            explainYourself(mode.Reset);
            stageNo = "11 A";
            stageDescription = "Drice Up To Zone A";
            explainYourself(mode.Transmit);

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end driveToZone_A


    // SCRIPT STAGE 11.5 A
    // Specialist Segment
    // Drive slowly until the front right color sensor see the blue tape
    public void driveToZone_A_stage_2(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Method Set up code goes here

            // Telemetry
            explainYourself(mode.Reset);
            stageNo = "11.5 A";
            stageDescription = "Dive Carefully Atop Zone A";
            explainYourself(mode.Transmit);

            // Check if its safe to run this method
            while (opModeIsActive() && (segmentTime.seconds() < segmentTimeLimit)) {

                // do stuff

                if ((robot.getFRColor_blue() < 300)) {  // && (!allDone)

                    robot.MecanumDrive(-0.2, 0, 0);

                } else {

                    robot.MecanumDrive(0, 0, 0);
                    //allDone = true;
                    break;
                }

                // update time telemetry readout
                explainYourself(mode.Transmit);
            }

        }
    } // end driveToZone_A_stage_2


    // Postscript Stage 12_A
    // Drive Segment
    // Pivot for Zone A
    public void pivotForZoneA(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .2;
            double FL_Distance = 21; // was 22
            double FR_distance = -21;
            double BL_distance = 21;
            double BR_distance = -21;

            // Telemetry
            explainYourself(mode.Reset);
            stageNo = "12 A";
            stageDescription = "Pivot for Zone A";
            explainYourself(mode.Transmit);

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end pivotForZoneA


    // Post-Script 13_A
    // Drive Segment
    // Back up a bit to drop Wobble targe fully into Zone A
    public void backIntoZoneA(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .3;
            double FL_Distance = 11; // was -11.5
            double FR_distance = 11;
            double BL_distance = 11;
            double BR_distance = 11;

            // Telemetry
            explainYourself(mode.Reset);
            stageNo = "13 A";
            stageDescription = "Back in Zone A";
            explainYourself(mode.Transmit);

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end backIntoZoneA


    // Postscript Stage 20_Z (and several other post script locations)
    // Specialist Segment
    // Lower La Chicken Wing
    public void lowerChickenWing(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Method Set up code goes here

            // Telemetry
            explainYourself(mode.Reset);
            stageNo = "20 Z";
            stageDescription = "Lower La Chicken Wing";
            explainYourself(mode.Transmit);

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
                    break;  // try a break statement here
                }

                // update time telemetry readout
                explainYourself(mode.Transmit);
            }

        }
    } // end lowerChickenWing


    // Postscript Stage 21_Z
    // Specialsit Segment
    // Release the Wobble Target
    public void releaseWobbleTarget(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Method Set up code goes here

            // Telemetry
            explainYourself(mode.Reset);
            stageNo = "21 Z";
            stageDescription = "Release Wobble Target";
            explainYourself(mode.Transmit);

            // Check if its safe to run this method
            while (opModeIsActive() && (segmentTime.seconds() < segmentTimeLimit)) {

                // Drop target
                robot.FingerGrab(.6);

                // update time telemetry readout
                explainYourself(mode.Transmit);

                break;
            }

        }
    } //End drop wobble target.

    // Post-Script Stage 22_Z
    // Specialist Stage
    // Raise La Chicken Wing
    public void raiseChickenWing(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Telemetry
            explainYourself(mode.Reset);
            stageNo = "22 Z";
            stageDescription = "Raise La Chicken Wing";
            explainYourself(mode.Transmit);

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
                    // break;  // try a break statement here
                }


                // update time telemetry readout
                explainYourself(mode.Transmit);
            }

        }
    } // end raiseChickenWing


    /**************************
     *                        *
     *  Autonomous  Segments  *
     *                        *
     *    POSTSCRIPT B        *
     *                        *
     **************************/

    // POSTSCRIPT STAGE 10_B_C
    // Drive Segment
    // Pivot to both Zones B and C
    public void pivot_B_C(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .3;
            double FL_Distance = 16.2;
            double FR_distance = -16.2;
            double BL_distance = 16.2;
            double BR_distance = -16.2;

            // Telemetry
            explainYourself(mode.Reset);
            stageNo = "10 B-C";
            stageDescription = "Pivot to Zones B+C";
            explainYourself(mode.Transmit);

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end pivot_B_C


    // STAGE 11 B
    // Drive Segment
    // Drive to Zone B
    public void driveToZone_B(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .3;
            double FL_Distance = -34;
            double FR_distance = -34;
            double BL_distance = -34;
            double BR_distance = -34;

            // Telemetry
            explainYourself(mode.Reset);
            stageNo = "11 B";
            stageDescription = "Drive to Zones B";
            explainYourself(mode.Transmit);

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end driveToZone_B


    // Postscript Stage 23_B
    // Drive segment
    // Strafe to the White line
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
            explainYourself(mode.Reset);
            stageNo = "23 B";
            stageDescription = "Strafe for White Line";
            explainYourself(mode.Transmit);

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end strafeFor_B_ToWhiteLine


    /**************************
     *                        *
     *  Autonomous  Segments  *
     *                        *
     *    POSTSCRIPT C        *
     *                        *
     **************************/

    public void driveNearWall(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .5;
            double FL_Distance = -30;
            double FR_distance = -30;
            double BL_distance = -30;
            double BR_distance = -30;

            // Telemetry
            explainYourself(mode.Reset);
            stageNo = "9999";
            stageDescription = "Words";
            explainYourself(mode.Transmit);

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end driveNearWall

    //Drive Segment
    public void driveCloseToWall(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Method Set up code goes here

            // Telemetry
            explainYourself(mode.Reset);
            stageNo = "8888";
            stageDescription = "Utterances";
            explainYourself(mode.Transmit);

            // Check if its safe to run this method
            while (opModeIsActive() && (segmentTime.seconds() < segmentTimeLimit)) {

                // do stuff

                if(robot.getFrontDistance() > 10)
                {
                    robot.FLDrive.setPower(-0.15); // whoop
                    robot.FRDrive.setPower(-0.15);
                    robot.BLDrive.setPower(-0.15);
                    robot.BRDrive.setPower(-0.15);
                }
                else
                {
                    robot.FLDrive.setPower(0);
                    robot.FRDrive.setPower(0);
                    robot.BLDrive.setPower(0);
                    robot.BRDrive.setPower(0);
                }
                // update time telemetry readout
                explainYourself(mode.Transmit);
            }

        }
    } // end DriveCloseToWall

    // Drive Segment
    public void turnToStrafeC(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .5;
            double FL_Distance = -42.5;
            double FR_distance = 42.5;
            double BL_distance = -42.5;
            double BR_distance = 42.5;


            // Telemetry
            explainYourself(mode.Reset);
            stageNo = "9999";
            stageDescription = "Words";
            explainYourself(mode.Transmit);

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end turnToStrafeC

    // Drive Segment
    public void backIntoWall(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .2;
            double FL_Distance = 7.5;
            double FR_distance = 7.5;
            double BL_distance = 7.5;
            double BR_distance = 7.5;

            // Telemetry
            explainYourself(mode.Reset);
            stageNo = "9999";
            stageDescription = "Words";
            explainYourself(mode.Transmit);
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);;
        }
    } // end strafeForC

    // Drive Segment
    public void rollForward(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .2;
            double FL_Distance = -3;
            double FR_distance = -3;
            double BL_distance = -3;
            double BR_distance = -3;

            // Telemetry
            explainYourself(mode.Reset);
            stageNo = "9999";
            stageDescription = "Words";
            explainYourself(mode.Transmit);
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);;
        }
    } // end strafeForC

    // Drive Segment
    public void strafeForC(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Drive Targets
            double speed = .2;
            double FL_Distance = -35;
            double FR_distance = 35;
            double BL_distance = 35;
            double BR_distance = -35;

            // Telemetry
            explainYourself(mode.Reset);
            stageNo = "9999";
            stageDescription = "Words";
            explainYourself(mode.Transmit);
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);;
        }
    } // end strafeForC

    /**************************
     *                        *
     *  OFF SCRIPT UTILITIES  *
     *                        *
     **************************/

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
    // Short description
    public void specialist_segment_template(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset the segment timer
            segmentTime.reset();

            // Method Set up code goes here

            // Telemetry
            explainYourself(mode.Reset);
            stageNo = "8888";
            stageDescription = "Utterances";
            explainYourself(mode.Transmit);

            // Check if its safe to run this method
            while (opModeIsActive() && (segmentTime.seconds() < segmentTimeLimit)) {

                // do stuff

                // update time telemetry readout
                explainYourself(mode.Transmit);
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
            explainYourself(mode.Reset);
            stageNo = "9999";
            stageDescription = "Words";
            explainYourself(mode.Transmit);

            // call encoderDrive
            encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance, segmentTimeLimit);

        }
    } // end drive template
}  // end class Auton5663_eocv