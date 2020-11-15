// Version 1.9

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Robot {

    /* Establish Essential Robot Modes */
    private int driveDirectionModifyer = 1;
    private boolean forceFieldOn = false;

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
   public DcMotor Intake = null;
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

    } // end constructor method Robot



    /****************
     * Hardware Map *
     ****************/
    public void hMap(HardwareMap hardwareMap) {

        //Drive Motors
        FLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
        FRDrive = hardwareMap.get(DcMotor.class, "FRDrive");
        BRDrive = hardwareMap.get(DcMotor.class, "BRDrive");
        BLDrive = hardwareMap.get(DcMotor.class, "BLDrive");
        //Setting Drive DC Motors direction
        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        BRDrive.setDirection(DcMotor.Direction.FORWARD);

        //Shooter
        LShooter = hardwareMap.get(DcMotor.class, "LShooter");
        RShooter = hardwareMap.get(DcMotor.class, "RShooter");
        // Setting Shooter DC Motors Direction
        RShooter.setDirection(DcMotor.Direction.FORWARD);
        LShooter.setDirection(DcMotor.Direction.REVERSE);

        //Ring Intake
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        //FeedBelt = hardwareMap.get(DcMotor.class, "FeedBelt");
        //Setting Ring Intake DC Motor direction
        Intake.setDirection(DcMotor.Direction.REVERSE);
        //FeedBelt.setDirection(DcMotor.Direction.FORWARD);

        // Wobble target arm
        //Lift = hardwareMap.get(DcMotor.class, "Lift");
        //Gripper = hardwareMap.get(Servo.class, "Gripper");
        // Setting Lift DC Motor Direction
        //Lift.setDirection(DcMotor.Direction.FORWARD);

        // Sensors
        FrontDistanceSensor = hardwareMap.get(DistanceSensor.class,"FrontDistanceSensor");

    } // end hMap


    /************************
     *        SETTER        *
     *  Toggle the value of *
     *  forwardDriveMode    *
     ************************/
    public void setForwardDriveMode () {
        if (driveDirectionModifyer == 1) {
            driveDirectionModifyer = -1;
        } else {
            driveDirectionModifyer = 1;
        }
    }

    /************************
     *        GETTER        *
     *  report the value of *
     *  forwardDriveMode    *
     ************************/
    public String getDirectionMode () {
        if (driveDirectionModifyer == 1) {
            return "FORWARD drive mode";
        } else {
            return "REVERSE drive mode";
        }
    }


    /************************
     *        SETTER        *
     *  Toggle the value of *
     *  forceFieldOn        *
     ************************/
    public void toggleForceField() {

        if (forceFieldOn) {
            forceFieldOn = false;
        } else {
            forceFieldOn = true;
        }
    }

    /************************
     *        GETTER        *
     *  report the value of *
     *  forceFieldOn        *
     ************************/
    public String isForceFieldOn() {

        if (forceFieldOn) {
            return "ON";
        } else {
            return "OFF";
        }
    }


    /************************
     *        GETTER        *
     *  report the value of *
     *  FrontDistanceSensor *
     ************************/
    public double getFrontDistance () {

        return FrontDistanceSensor.getDistance(DistanceUnit.INCH);

    }



    /*************************
     *  CarWash - start the  *
     *  rear ring intake     *
     *  carwash spinner      *
     *************************/
    public void CarWash (double speed) {

        Intake.setPower(speed);

    }



    /******************************************
     * MecanumDrive method responsible of all *
     * drive motions: drive, strafe, and turn *
     ******************************************/
    public void MecanumDrive(double drive, double strafe, double turn) {
        double FLPower;
        double FRPower;
        double BLPower;
        double BRPower;

        double masterPowerLimit;
        masterPowerLimit = .6;

        double driveAdjust;
        double turnAdjust;
        double strafeAdjust;

        if (forceFieldOn) {
            driveAdjust = 0;
            turnAdjust = 0;
            strafeAdjust = masterPowerLimit / 2;
        } else {
            driveAdjust = masterPowerLimit;
            turnAdjust = masterPowerLimit;
            strafeAdjust = masterPowerLimit;
        }

        FRPower = ((drive * driveAdjust * driveDirectionModifyer) + (strafe * strafeAdjust * driveDirectionModifyer) + (turn * turnAdjust)) ;
        FLPower = ((drive * driveAdjust * driveDirectionModifyer) - (strafe * strafeAdjust * driveDirectionModifyer) - (turn * turnAdjust)) ;
        BRPower = ((drive * driveAdjust * driveDirectionModifyer) - (strafe * strafeAdjust * driveDirectionModifyer) + (turn * turnAdjust)) ;
        BLPower = ((drive * driveAdjust * driveDirectionModifyer) + (strafe * strafeAdjust * driveDirectionModifyer) - (turn * turnAdjust)) ;

        FLDrive.setPower(FLPower);
        FRDrive.setPower(FRPower);
        BLDrive.setPower(BLPower);
        BRDrive.setPower(BRPower);

    } // end MechanumDrive


    /***********************************************
     * Shooter method responsible for Ring Shooter *
     * control. Current method implements two      *
     * launching velocities                        *
     ***********************************************/
    public void Shooter(double shootPower) {

       RShooter.setPower(shootPower);
       LShooter.setPower(shootPower);

    } // end Shooter

} // end Class Robot



