package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.logitech.LogitechGamepadF310;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleOp5663", group="Iterative Opmode")
public class TeleOp5663 extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    Robot robot = new Robot();

    private Servo Gripper = null;

    /***********************************************
     * This method is run ONCE when drivers hits INIT
     ***********************************************/
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.hMap(hardwareMap);
    }


    /***********************************************
     * Code to run REPEATEDLY after the driver
     * hits INIT, but before they hit PLAY
     ***********************************************/
    @Override
    public void init_loop(){


    }


    /***********************************************
     * Code to run ONCE when the driver hits PLAY
     ***********************************************/
    @Override
    public void start(){
        runtime.reset();

    }


    /***********************************************
     * Code to run REPEATEDLY after the driver
     * hits PLAY but before they hit STOP
     ***********************************************/
    @Override
    public void loop(){
        // assign pilot controler valure to appropriate variables
        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.right_stick_x;
        double turn = gamepad1.left_stick_x;

        // call drive and shooter methods with gamepad input
        robot.MecanimDrive(drive, strafe, turn);
        robot.Shooter(gamepad2.right_trigger > 0, gamepad2.left_trigger > 0);

        // provide telemetry feedback to drivers
        telemetry.addData("LShooter", robot.LShooter.getCurrentPosition());
        telemetry.addData("RShooter", robot.RShooter.getCurrentPosition());
        telemetry.addData("FRDrive", robot.FRDrive.getCurrentPosition());
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }


    /***********************************************
     * Code to run ONCE after the driver hits STOP
     ***********************************************/
    @Override
    public void stop() {


    }
}
