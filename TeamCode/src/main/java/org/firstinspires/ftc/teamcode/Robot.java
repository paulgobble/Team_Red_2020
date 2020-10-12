package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

public class Robot {
    /* Create Elapsed runtimer */
    private ElapsedTime runtime = new ElapsedTime();

    /* Create DcMotors for drive */
    public DcMotor FLDrive = null;
    public DcMotor FRDrive = null;
    public DcMotor BLDrive = null;
    public DcMotor BRDrive = null;

    /* Create DcMotors for shooter */
    public DcMotor LShooter = null;
    public DcMotor RShooter = null;

    /* Create other motors and servos */
    public Servo Gripper = null;
    //Lift is the new variable
    //Picker is the new variable

    /* Create Hardware Map */
    HardwareMap hMap =  null;

    /**********************
     /* Create constructor *
     **********************/
    public void Robot() {

    }


    /****************
     * Hardware Map *
     ****************/
    public void hMap(HardwareMap hardwareMap) {
        FLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
        FRDrive = hardwareMap.get(DcMotor.class, "FRDrive");
        BRDrive = hardwareMap.get(DcMotor.class, "BRDrive");
        BLDrive = hardwareMap.get(DcMotor.class, "BLDrive");
        Gripper = hardwareMap.get(Servo.class, "Gripper");
        LShooter = hardwareMap.get(DcMotor.class, "LShooter");
        RShooter = hardwareMap.get(DcMotor.class, "RShooter");

        FLDrive.setDirection(DcMotor.Direction.FORWARD);
        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);
        RShooter.setDirection(DcMotor.Direction.FORWARD);
        LShooter.setDirection(DcMotor.Direction.REVERSE);
    }


    /******************************************
     * MecanimDrive method responsible of all *
     * drive motions: drive, strafe, and turn *
     ******************************************/
    public void MecanimDrive(double drive, double strafe, double turn) {
        double FLPower;
        double FRPower;
        double BLPower;
        double BRPower;

        FRPower = -strafe + drive - turn;
        FLPower = strafe + drive + turn;
        BRPower = strafe + drive - turn;
        BLPower = -strafe + drive + turn;

        FLDrive.setPower(FLPower);
        FRDrive.setPower(FRPower);
        BLDrive.setPower(BLPower);
        BRDrive.setPower(BRPower);

    }


    /***********************************************
     * Shooter method responsible for Ring Shooter *
     * controle. Current method implements two     *
     * launching velocities                        *
     ***********************************************/
    public void Shooter(boolean in, boolean out) {
        // duplicate left and right code needed to avoid odd behiavior
        // Right shooter
        if (in && !out) {
            RShooter.setPower(0.75);
        } else if (out && !in) {
            RShooter.setPower(0.5);
        } else
        {
            RShooter.setPower(0);
        }

        // Left shooter
        if (in && !out)
        {
            LShooter.setPower(0.75);
        } else if (out && !in)
        {
            LShooter.setPower(0.5);
        }
        else {
            LShooter.setPower(0);
        }
    }
}