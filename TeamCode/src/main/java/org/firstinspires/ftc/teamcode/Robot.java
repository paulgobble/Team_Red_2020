// Version 1.3

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
   // public Servo Gripper = null;
   // public DcMotor Intake = null;
    //public DcMotor FeedBelt = null;
   // public DcMotor Lift = null;
    //Lift is the new variable
    //Picker is the new variable

    /* Create sensors */
    public DistanceSensor FrontDistanceSensor = null; // NEW

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


        //Drive Motors
        FLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
        FRDrive = hardwareMap.get(DcMotor.class, "FRDrive");
        BRDrive = hardwareMap.get(DcMotor.class, "BRDrive");
        BLDrive = hardwareMap.get(DcMotor.class, "BLDrive");


        //Arm and intake
        //Gripper = hardwareMap.get(Servo.class, "Gripper");
        //Intake = hardwareMap.get(DcMotor.class, "Intake");
        //FeedBelt = hardwareMap.get(DcMotor.class, "FeedBelt");
        //Lift = hardwareMap.get(DcMotor.class, "Lift");

        //Shooter
        LShooter = hardwareMap.get(DcMotor.class, "LShooter");
        RShooter = hardwareMap.get(DcMotor.class, "RShooter");

        FLDrive.setDirection(DcMotor.Direction.FORWARD);
        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);
        RShooter.setDirection(DcMotor.Direction.FORWARD);
        LShooter.setDirection(DcMotor.Direction.REVERSE);

        // Sensors
        FrontDistanceSensor = hardwareMap.get(DistanceSensor.class,"FrontDistanceSensor"); // NEW
    }


    /******************************************
     * MecanimDrive method responsible of all *
     * drive motions: drive, strafe, and turn *
     ******************************************/
    public void MecanumDrive(double drive, double strafe, double turn, boolean forceFieldOn) {
        double FLPower;
        double FRPower;
        double BLPower;
        double BRPower;

        int driveAdjust = 1;
        int turnAdjust = 1;
        int strafeAdjust = 1;

        if (forceFieldOn) {
            driveAdjust = 0;
            turnAdjust = 0;
        } else {
            driveAdjust = 1;
            turnAdjust = 1;
        }

        FRPower = (-strafe * strafeAdjust) + (drive * driveAdjust) - (turn * turnAdjust);
        FLPower = (strafe * strafeAdjust) + (drive * driveAdjust) + (turn * turnAdjust);
        BRPower = (strafe * strafeAdjust) + (drive * driveAdjust) - (turn * turnAdjust);
        BLPower = (-strafe * strafeAdjust) + (drive * driveAdjust) + (turn * turnAdjust);

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
    public void Shooter(boolean FastShoot, boolean SlowShoot) {
        // duplicate left and right code needed to avoid odd behiavior
        // Right shooter
        if (FastShoot && !SlowShoot) {
            RShooter.setPower(0.75);
        } else if (SlowShoot && !FastShoot) {
            RShooter.setPower(0.5);
        } else
        {
            RShooter.setPower(0);
        }

        // Left shooter
        if (FastShoot && !SlowShoot)
        {
            LShooter.setPower(0.75);
        } else if (SlowShoot && !FastShoot)
        {
            LShooter.setPower(0.5);
        }
        else {
            LShooter.setPower(0);
        }

        //distance sensor getting


        }



    }



