// Version 1.9.8

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auton5663_eocv", group="Autonomous")
//@Disabled
public class Auton5663_eocv extends LinearOpMode {

    /* Declare OpMode members. */
    Robot robot   = new Robot();
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime segmentTime = new ElapsedTime();

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


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        runtime.reset();

        watchAndReport(30);

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
