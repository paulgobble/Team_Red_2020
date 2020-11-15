// Version 1.8.1

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



    /*************************
     *  CarWash - start the  *
     *  rear ring intake     *
     *  carwash spinner      *
     *************************/
    public void CarWash (double speed) {

        Intake.setPower(speed);

    }



    /******************
     *  ReportStatus  *
     ******************/
    public void ReportStatus() {

        //telemetry.addData("Robot", "Woff Woff");  // all calls to the telemetry.addData method crash running program
        //telemetry.addData(theCaption, theMessage);
        //telemetry.addData("FrontDistanceSensor", FrontDistanceSensor.getDistance(DistanceUnit.INCH));
        // Drive Motors
        //telemetry.addData("FRDrive", FRDrive.getCurrentPosition());
        //telemetry.addData("FLDrive", FLDrive.getCurrentPosition());
        //telemetry.addData("BRDrive", BRDrive.getCurrentPosition());
        //telemetry.addData("BLDrive", BLDrive.getCurrentPosition());
        // Shooter motors
        //telemetry.addData("RShooter", RShooter.getCurrentPosition());
        //telemetry.addData("LShooter", LShooter.getCurrentPosition());

    } // end ReportStatus



    /******************************************
     * MecanumDrive method responsible of all *
     * drive motions: drive, strafe, and turn *
     ******************************************/
    public void MecanumDrive(double drive, double strafe, double turn, boolean forceFieldOn) {
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

        FRPower = (drive * driveAdjust) + (strafe * strafeAdjust) + (turn * turnAdjust);
        FLPower = (drive * driveAdjust) - (strafe * strafeAdjust) - (turn * turnAdjust);
        BRPower = (drive * driveAdjust) - (strafe * strafeAdjust) + (turn * turnAdjust);
        BLPower = (drive * driveAdjust) + (strafe * strafeAdjust) - (turn * turnAdjust);

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



