// Version 1.9 eocv.2

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class Robot {

    /* Establish Essential Robot Modes */
    private int driveDirectionModifyer = 1;     // What do we consider "Forward" end of Redbot
    private boolean forceFieldArmed = false;    // Has the force field system been activated by pilot
    private boolean forceFieldTriggered = false;  // Has the Robot turned on the force field 'cause it wsa activated by pilot and roobt is within the stop distance
    private final double stopDistance = 14.0;
    private boolean streamingVideo;             // is the connection to the webcam open

    /* Define Enumerator for possible TargetZones*/
    enum TargetZones {
        A,
        B,
        C
    }

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
    private DcMotor Intake = null;
    // public DcMotor Lift = null;
    // public Servo Gripper = null;

    /* Create sensors */
    private DistanceSensor FrontDistanceSensor = null; // NEW

    private RevBlinkinLedDriver blinkinLedDriver = null;
    private RevBlinkinLedDriver.BlinkinPattern pattern = null;

    /* Create a webcam */
    OpenCvCamera webcam;

    /* Create Hardware Map */
    HardwareMap hMap =  null;


    //public Robot() {  // this method create twice?
    //}


    /***********************
     *  Create constructor *
     ***********************/
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
        //Lift = hardwareMap.get(DcMotor.class, "Lift");
        //Gripper = hardwareMap.get(Servo.class, "Gripper");
        // Setting Lift DC Motor Direction
        //Lift.setDirection(DcMotor.Direction.FORWARD);

        // Sensors
        FrontDistanceSensor = hardwareMap.get(DistanceSensor.class,"FrontDistanceSensor");
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        // webcam
        if(streamingVideo) {
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
            webcam.setPipeline(new RedPipeline());
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }
            });

        }

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


    /************************
     *        SETTER        *
     *  Set the value of    *
     *  streamingVideo      *
     ************************/
    public void setStreamingVideo(boolean onOrOff){
        streamingVideo = onOrOff;
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

        if ((forceFieldArmed) && (forceFieldTriggered)) {
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


    /************************************
     *  Method directing Robot to check *
     *  its sensors, adjust the
     *  force field and set lights      *
     ************************************/
    public void updateFFStatus() {

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


    /********************************
     *  Method directing Robot to   *
     *  change its LEDs to indicate *
     *  the detected target zone    *
     ********************************/
    public void idTargetZone(TargetZones passedZone){
        switch (passedZone){
            case A:
                pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                break;
            case B:
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
                break;
            case C:
                pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                break;
        }

        blinkinLedDriver.setPattern(pattern);

    } // end idTargetZone


    /******************************
     *  Nested Class defining the *
     *  EasyOpenCV pipeline       *
     ******************************/
    class RedPipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input)
        {

            /*
             * Draw a simple box around the area where we expect the target rings to be seen
             */
            Imgproc.rectangle(
                    input,
                    new Point(
                            120,
                            50),
                    new Point(
                            input.cols()-120,
                            input.rows()-140),
                    new Scalar(255, 0, 0), 4);

            return input;
        }

        @Override
        public void onViewportTapped()
        {

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    } // end nested Class RedPipeline



} // end Class Robot



