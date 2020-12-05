// Version 1.9 eocv.2

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

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.setStreamingVideo(true);

        robot.hMap(hardwareMap);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        watchAndReport(10, org.firstinspires.ftc.teamcode.Robot.TargetZones.A);
        watchAndReport(10, org.firstinspires.ftc.teamcode.Robot.TargetZones.B);
        watchAndReport(10, org.firstinspires.ftc.teamcode.Robot.TargetZones.C);

    } // end runOpMode


    public void watchAndReport(double timeoutS, org.firstinspires.ftc.teamcode.Robot.TargetZones thisZone) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // reset the timeout time
            runtime.reset();

            while (opModeIsActive() && (runtime.seconds() < timeoutS)) {

                // Display it for the driver.
                telemetry.addData("Timer",  runtime.seconds());
                telemetry.update();
                robot.idTargetZone(thisZone);
            }
        }
    } // end watchAndReport

}  // end class Auton5663_eocv
