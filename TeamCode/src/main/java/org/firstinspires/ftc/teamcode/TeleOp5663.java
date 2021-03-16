// Version 2.0

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name="TeleOp5663", group="Iterative Opmode")
public class TeleOp5663 extends OpMode
{
    Robot robot = new Robot();

    private ElapsedTime runtime = new ElapsedTime(); //do we need ElaspedTime in both Robot and TeleOP

    /* Rate limit gamepad button presses to every 500ms. */
    private final static int ButtonLockout = 500;

    private Deadline buttonPressLimit;


    /***********************************************
     * This method is run ONCE when drivers hits INIT
     ***********************************************/
    @Override
    public void init() {
        robot.setStreamingVideo(false);

        robot.hMap(hardwareMap);

        telemetry.addData("Status:", "Initialized v2.0");
        telemetry.addData("Drive Orientation:", "Forward");
        telemetry.addData("Force Field:", "Off");
        telemetry.addData("Front Distance:", "-------");

        telemetry.addData("FLDrive:",robot.FLDrive.getCurrentPosition());
        telemetry.addData("FRDrive:", robot.FRDrive.getCurrentPosition());
        telemetry.addData("BLDrive:", robot.BLDrive.getCurrentPosition());
        telemetry.addData("BRDrive:", robot.BRDrive.getCurrentPosition());

        buttonPressLimit = new Deadline(ButtonLockout, TimeUnit.MILLISECONDS);

    } // end init


    /***********************************************
     * Code to run REPEATEDLY after the driver
     * hits INIT, but before they hit PLAY
     ***********************************************/
    @Override
    public void init_loop(){

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

    } // end init_loop


    /***********************************************
     * Code to run ONCE when the driver hits PLAY
     ***********************************************/
    @Override
    public void start(){
        runtime.reset();
        telemetry.addData("Status", "S T A R T !!");

        robot.LaChickenWing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LaChickenWing.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.FingerGrab(1);
        //robot.ReportStatus("Status:", "S T A R T !");
    } // end start


    /***********************************************
     * Code to run REPEATEDLY after the driver
     * hits PLAY but before they hit STOP
     ***********************************************/
    @Override
    public void loop(){


        telemetry.addData("Current Wing Position", robot.LaChickenWing.getCurrentPosition());
        // handle toggle button input
        handleButtons();

        // Have Robot update its status
        robot.updateFFStatus();

        // Report Telemetry Data
        telemetry.addData("Elasped Time:", "%.1f", runtime.seconds());
        telemetry.addData("Drive Orientation:", robot.getDirectionMode());
        telemetry.addData("Forcefield:", robot.isForceFieldOn());
        telemetry.addData("Front Distance:","%.1f", robot.getFrontDistance());

        //Send telemetry message to monitor encoder values
        telemetry.addData("FLDrive:",robot.FLDrive.getCurrentPosition());
        telemetry.addData("FRDrive:", robot.FRDrive.getCurrentPosition());
        telemetry.addData("BLDrive:", robot.BLDrive.getCurrentPosition());
        telemetry.addData("BRDrive:", robot.BRDrive.getCurrentPosition());

        telemetry.update();

        // Input, compute, and send drive input data
        double driveNormal = gamepad1.left_stick_y; // Drive value entered on the left "normal drive" joystick
        double driveCreep = gamepad1.right_stick_y; // Drive value entered on the right "creep" joystick
        double driveResult;                         // Computed drive value sent to Robot

        if (driveCreep == 0) {
            driveResult = driveNormal;
        } else {
            driveResult = driveCreep * .5;
        }

        double strafe = gamepad1.right_stick_x;
        double turn = gamepad1.left_stick_x;

        // call Robot's mecanumDrive method
        robot.MecanumDrive(driveResult, strafe, turn);


        // Input, compute, and send shooter input data
        double shootFast = gamepad2.right_trigger;  // Input from Shoot Fast Trigger
        double shootSlow = gamepad2.left_trigger;   // Input from Shoot Slow Trigger
        double shootPower = 0;                      // Computed shooting power sent to Robot

        if (shootFast != 0) {
            shootPower = .65;
        } else if (shootSlow != 0) {
            shootPower = .5;
        } else {
            shootPower = -0.2;
        }
        robot.Shooter(shootPower);

        // Turn on or off the rear Ring Intake carwash spinner
        if (gamepad2.x) {
            robot.CarWash(.9);
        } else {
            robot.CarWash((0));
        }

        // La Chicken Wing D-pod control
        boolean wingFlapUp = gamepad2.dpad_up;
        boolean wingFlapDown = gamepad2.dpad_down;
        double wingSpeed = .5;
        if (wingFlapUp) {
            //raise the wing
            robot.FlapWing(wingSpeed);
        } else if (wingFlapDown) {
            // lower the wing
            robot.FlapWing(wingSpeed * -1);
            robot.FingerGrab(1);
        } else {
            // do nothing
            robot.FlapWing(0);
        }

        // The Chicken Finger D-pad Controle
        boolean fingerOpen = gamepad2.dpad_left;
        boolean fingegClose = gamepad2.dpad_right;
        if(fingerOpen) {
            robot.FingerGrab(.6);
        } else if (fingegClose){
            robot.FingerGrab(1);
        }



    } // end loop


    /***********************************************
     * Code to run ONCE after the driver hits STOP *
     ***********************************************/
    @Override
    public void stop() {


    } // end stop


    /*******************************************
     * Method to handle gamepad toggle buttons *
     * presses and debounce                    *
     *******************************************/
    private void handleButtons () {

        // check if we've waited long enough
        if (!buttonPressLimit.hasExpired()) {
            return;
        }

        // Set Robot Options
        if (gamepad1.x) {
            robot.setForwardDriveMode();
            buttonPressLimit.reset();
        } else if (gamepad1.b) {
            //robot.toggleForceField();
            //buttonPressLimit.reset();
        }

    } // End handleButtons


} // End class TeleOp5663
