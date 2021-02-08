// Version 1.9 eocv.3

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

    @Override
    public void runOpMode() {

        telemetry.addData("Codebase", "v 1.9 eocv.3");

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

        watchAndReport(30);

    } // end runOpMode


    public void watchAndReport(double timeoutS) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // reset the timeout time
            runtime.reset();

            while (opModeIsActive() && (runtime.seconds() < timeoutS)) {

                // Display it for the driver.
                telemetry.addData("Time:",  runtime.seconds());
                telemetry.addData("TZAV:", robot.getTargetZoneAverageValue());
                telemetry.addData("Decifered Target Zone", robot.getDesciferedTargetZone());
                telemetry.update();
            }
        }
    } // end watchAndReport

}  // end class Auton5663_eocv
