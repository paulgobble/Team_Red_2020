// Version 1.9.3

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Robot {

    /* Establish Essential Robot Modes */
    private int driveDirectionModifyer = 1;     // What do we consider "Forward"
    private boolean forceFieldArmed = false;    // Has the force field system been activated by pilot
    private boolean inDangerZone = false;       // Is the robot within the stop distance limit
    private boolean forceFieldTriggered = false;  // Has the Robot turned on the force field 'cause it wsa activated by pilot and robot is within the stop distance
    private final double stopDistance = 14.0;

    /* Create Elapsed runtimer */
    private ElapsedTime runtime = new ElapsedTime();

    /* Create DcMotors for drive */
    private DcMotor FLDrive = null;
    private DcMotor FRDrive = null;
    private DcMotor BLDrive = null;
    private DcMotor BRDrive = null;

    /* Create DcMotors for shooter */
    private DcMotor LShooter = null;
    private DcMotor RShooter = null;

    /* Create other motors and servos */
    // public Servo Gripper = null;
    private DcMotor Intake = null;
    //public DcMotor FeedBelt = null;
    public DcMotor Lift = null;
    //Lift is the new variable
    //Picker is the new variable

    /* Create sensors */
    private DistanceSensor FrontDistanceSensor = null; // NEW

    private RevBlinkinLedDriver blinkinLedDriver = null;
    private RevBlinkinLedDriver.BlinkinPattern pattern = null;



    /* Create Hardware Map */
    HardwareMap hMap =  null;

    public Robot() {
    }


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
        //Setting Ring Intake DC Motor direction
        Intake.setDirection(DcMotor.Direction.REVERSE);
        //FeedBelt.setDirection(DcMotor.Direction.FORWARD);

        // Wobble target arm
        Lift = hardwareMap.get(DcMotor.class, "Lift");
        //Gripper = hardwareMap.get(Servo.class, "Gripper");
        // Setting Lift DC Motor Direction
        //Lift.setDirection(DcMotor.Direction.FORWARD);

        // Sensors
        FrontDistanceSensor = hardwareMap.get(DistanceSensor.class,"FrontDistanceSensor");
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

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
            return "FORWARD (Shooter)";
        } else {
            return "REVERSE (Wedge)";
        }
    }


    /************************
     *        SETTER        *
     *  Toggle the value of *
     *  forceFieldOn        *
     ************************/
    public void toggleForceField() {

        if (forceFieldArmed) {
            forceFieldArmed = false;
        } else {
            forceFieldArmed = true;
        }
    }

    /************************
     *        GETTER        *
     *  report the value of *
     *  forceFieldOn        *
     ************************/
    public String isForceFieldOn() {

        if (forceFieldTriggered) {
            return "ON";
        } else if (forceFieldArmed) {
            return "Armed";
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

        //The powers for the wheels
        double FLPower;
        double FRPower;
        double BLPower;
        double BRPower;

        double masterPowerLimit;
        masterPowerLimit = .6;

        double driveAdjust;
        double turnAdjust;
        double strafeAdjust;


        double speed = 5;


        if ((forceFieldArmed) && (forceFieldTriggered)) {
            driveAdjust = 0;
            turnAdjust = 0;
            strafeAdjust = masterPowerLimit / 2;
        } else {
            driveAdjust = masterPowerLimit;
            turnAdjust = masterPowerLimit;
            strafeAdjust = masterPowerLimit;
        }

        FRPower = ((drive * driveAdjust * driveDirectionModifyer) * speed + (strafe * strafeAdjust * driveDirectionModifyer) * speed + (turn * turnAdjust) * speed) ;
        FLPower = ((drive * driveAdjust * driveDirectionModifyer) * speed - (strafe * strafeAdjust * driveDirectionModifyer) * speed - (turn * turnAdjust) * speed) ;
        BRPower = ((drive * driveAdjust * driveDirectionModifyer) * speed - (strafe * strafeAdjust * driveDirectionModifyer) * speed + (turn * turnAdjust) * speed) ;
        BLPower = ((drive * driveAdjust * driveDirectionModifyer) * speed + (strafe * strafeAdjust * driveDirectionModifyer) * speed - (turn * turnAdjust) * speed) ;

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


    /***********************************************
     * Raise or lower La Chicke WIng               *
     * wobble targert lifter                       *
     * Accepts a sing argument for DC motor spoeed *
     ***********************************************/
    public void FlapWing(double wingSpeed) {

        Lift.setPower(wingSpeed);

    }




    /************************************
     *  Method directing Robot to check *
     *  its sensors, adjust the
     *  force field and set lights      *
     ************************************/
    public void updateStatus () {

        // check if Robot should power of the force field
        if ((forceFieldArmed) && (getFrontDistance() <= stopDistance)) {
            forceFieldTriggered = true;
            pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        } else if ((forceFieldArmed) && (getFrontDistance()) > stopDistance){
            forceFieldTriggered = false;
            pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
        } else {
            forceFieldTriggered = false;
            pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        }
        blinkinLedDriver.setPattern(pattern);

    } // end updateStatus

} // end Class Robot



