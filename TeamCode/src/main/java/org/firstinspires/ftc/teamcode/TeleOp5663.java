<<<<<<< HEAD
// Version 1.3
=======
// Version 1.31
>>>>>>> 9ab601d513efe36eabe20fcbdd30d5a8c14117ca

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp5663", group="Iterative Opmode")
public class TeleOp5663 extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime(); //do we need ElaspedTime in both Robot and TeleOP

    Robot robot = new Robot();

    private boolean forceFieldOn = false;


    /***********************************************
     * This method is run ONCE when drivers hits INIT
     ***********************************************/
    @Override
    public void init() {
        robot.hMap(hardwareMap);
        telemetry.addData("Status:", "Initialized v1.8");
        //robot.ReportStatus("Status:", "Initialized v1.5.1");
    } // end init


    /***********************************************
     * Code to run REPEATEDLY after the driver
     * hits INIT, but before they hit PLAY
     ***********************************************/
    @Override
    public void init_loop(){


    } // end init_loop


    /***********************************************
     * Code to run ONCE when the driver hits PLAY
     ***********************************************/
    @Override
    public void start(){
        runtime.reset();
        telemetry.addData("Status:", "S T A R T !!");
        //robot.ReportStatus("Status:", "S T A R T !");
    } // end start


    /***********************************************
     * Code to run REPEATEDLY after the driver
     * hits PLAY but before they hit STOP
     ***********************************************/
    @Override
    public void loop(){

        // Ask Robot to note the elasped time using telemetry
        //robot.ReportStatus();                                 // This method in robot doesn't work.  I can't make telemetry.addData work form robot.java

        // Input, compute, and send drive input data
        double driveNormal = gamepad1.left_stick_y; // Drive value entered on the left "normal drive" joystick
        double driveCreep = gamepad1.right_stick_y; // Drive value entered on the right "creep" joystice
        double driveResult;                         // Computed drive value sent to Robot

        if (driveCreep == 0) {
            driveResult = driveNormal;
        } else {
            driveResult = driveCreep * .5;
        }

        double strafe = gamepad1.right_stick_x;
        double turn = gamepad1.left_stick_x;

        // call Robot's mecanumDrive method
        robot.MecanumDrive(driveResult, strafe, turn, forceFieldOn);


        // Input, compute, and send shooter input data
        double shootFast = gamepad2.right_trigger;  // Input from Shoot Fast Trigger
        double shootSlow = gamepad2.left_trigger;   // Input from Shoot Slow Trigger
        double shootPower = 0;                      // Computed shooting power sent to Robot

        if (shootFast != 0) {
            shootPower = .75;
        } else if (shootSlow != 0) {
            shootPower = .5;
        } else {
            shootPower = 0;
        }
        robot.Shooter(shootPower);

        // Turn on or off the rear Ring Intake carwash spinner
        if (gamepad2.x) {
            robot.CarWash(.5);
        } else {
            robot.CarWash((0));
        }


    } // end loop


    /***********************************************
     * Code to run ONCE after the driver hits STOP
     ***********************************************/
    @Override
    public void stop() {


    } // end stop

} // End class TeleOp5663
