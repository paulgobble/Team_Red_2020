// Version 1.9.8

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

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.setStreamingVideo(true);

        telemetry.addData("Video Streaming", "Started");
        telemetry.update();

        robot.hMap(hardwareMap);

        // paste code here
        //Tell the driver that the encoders are resetting.
        telemetry.addData("Status:", "Initialized v1.9.8");
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        // Reverse the motor that runs backwards when connected directly to the battery
        //robot.FRDrive.setDirection(DcMotor.Direction.FORWARD);
        //robot.FLDrive.setDirection(DcMotor.Direction.REVERSE);
        //robot.BRDrive.setDirection(DcMotor.Direction.FORWARD);
        //robot.BLDrive.setDirection(DcMotor.Direction.REVERSE);

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

        telemetry.addData("FLDrive:",robot.FLDrive.getCurrentPosition());
        telemetry.addData("FRDrive:", robot.FRDrive.getCurrentPosition());
        telemetry.addData("BLDrive:", robot.BLDrive.getCurrentPosition());
        telemetry.addData("BRDrive:", robot.BRDrive.getCurrentPosition());

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        runtime.reset();

        watchAndReport(3);

        prepChickenWing(10);

    } // end runOpMode


    public void watchAndReport(double segmentTimeLimit) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // reset the timeout time
            segmentTime.reset();

            while (opModeIsActive() && (segmentTime.seconds() < segmentTimeLimit)) {

                // Display it for the driver.
                telemetry.addData("Time:",  runtime.seconds());
                telemetry.addData("TZAV:", robot.getTargetZoneAverageValue());
                telemetry.addData("Deciphered Target Zone", robot.getDecipheredTargetZone());
                telemetry.update();
            }
        }
    } // end watchAndReport



}  // end class Auton5663_eocv
