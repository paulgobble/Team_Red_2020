// Version 1.9.4

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    /* Telemetry items */
    private Telemetry.Item TI_message;
    private Telemetry.Item TI_driveOrientation;
    private Telemetry.Item TI_forceFieldMode;
    private Telemetry.Item TI_frontDistance;




    /***********************************************
     * This method is run ONCE when drivers hits INIT
     ***********************************************/
    @Override
    public void init() {
        robot.hMap(hardwareMap);

        TI_message = telemetry.addData("Status:", "Initialized v1.9.4");
        TI_driveOrientation = telemetry.addData("Drive Orientation", "Forward");
        TI_forceFieldMode = telemetry.addData("Force Field", "Off");
        TI_frontDistance = telemetry.addData("Front Distance", "-------");

        buttonPressLimit = new Deadline(ButtonLockout, TimeUnit.MILLISECONDS);

        //robot.ReportStatus();
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
        TI_message = telemetry.addData("Status", "S T A R T !!");

        robot.FingerGrab(.6);
        //robot.ReportStatus("Status:", "S T A R T !");
    } // end start


    /***********************************************
     * Code to run REPEATEDLY after the driver
     * hits PLAY but before they hit STOP
     ***********************************************/
    @Override
    public void loop(){

        // handle toggle button input
        handleButtons();

        // Have Robot update its status
        robot.updateStatus();

        // Report Telemetry Data
        TI_message = telemetry.addData("Elasped Time", "%.1f", runtime.seconds());
        TI_driveOrientation = telemetry.addData("Drive Orientation:", robot.getDirectionMode());
        TI_forceFieldMode = telemetry.addData("Forcefield:", robot.isForceFieldOn());
        TI_frontDistance = telemetry.addData("Front Distance","%.1f", robot.getFrontDistance());

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
            shootPower = .75;
        } else if (shootSlow != 0) {
            shootPower = .5;
        } else {
            shootPower = 0;
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
            //rasie the wing
            robot.FlapWing(wingSpeed);
        } else if (wingFlapDown) {
            // lower the wing
            robot.FlapWing(wingSpeed * -1);
        } else {
            // do nothing
            robot.FlapWing(0);
        }

        // The Chicken Finger D-pad Controle
        boolean fingerOpen = gamepad2.dpad_left;
        boolean fingegClose = gamepad2.dpad_right;
        if(fingerOpen) {
            robot.FingerGrab(.2);
        } else if (fingegClose){
            robot.FingerGrab(.6);
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
            robot.toggleForceField();
            buttonPressLimit.reset();
        }

    } // End handleButtons


} // End class TeleOp5663
