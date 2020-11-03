// Version 1.3

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
        // assign pilot controler value to appropriate variables
        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.right_stick_x;
        double turn = gamepad1.left_stick_x;

        //distance sensor finder
        if (gamepad2.x) {
            if (robot.FrontDistanceSensor.getDistance(DistanceUnit.INCH) < 20) {
                telemetry.addData("Force Field", "ON");
                forceFieldOn = true;
            } else {
                telemetry.addData("Force Field", "ARMED");
                forceFieldOn = false;
            }
        } else {
            telemetry.addData("Force Field", "OFF");
            forceFieldOn = false;
        }

        // call drive and shooter methods with gamepad input
        robot.MecanumDrive(drive, strafe, turn, forceFieldOn);
        robot.Shooter(gamepad2.right_trigger > 0, gamepad2.left_trigger > 0);






        // provide telemetry feedback to drivers
        telemetry.addData("FrontDistanceSensor", robot.FrontDistanceSensor.getDistance(DistanceUnit.INCH)); // NEW
        //telemetry.addData("LShooter", robot.LShooter.getCurrentPosition());
        //telemetry.addData("RShooter", robot.RShooter.getCurrentPosition());
        //telemetry.addData("FRDrive", robot.FRDrive.getCurrentPosition());
        //telemetry.addData("FLDrive", robot.FLDrive.getCurrentPosition());
        //telemetry.addData("BRDrive", robot.BRDrive.getCurrentPosition());
        //telemetry.addData("BLDrive", robot.BLDrive.getCurrentPosition());
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }


    /***********************************************
     * Code to run ONCE after the driver hits STOP
     ***********************************************/
    @Override
    public void stop() {


    }
}
