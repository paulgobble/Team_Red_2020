// Version 1.9.8

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class Robot {

    /* Establish Essential Robot Modes */
    private int driveDirectionModifyer = 1;     // What do we consider "Forward"

    private boolean forceFieldArmed = false;    // Has the force field system been activated by pilot
    private boolean inDangerZone = false;       // Is the robot within the stop distance limit
    private boolean forceFieldTriggered = false;  // Has the Robot turned on the force field 'cause it wsa activated by pilot and robot is within the stop distance
    private final double stopDistance = 14.0;

    private boolean streamingVideo;             // is the connection to the webcam open

    /* Define Enumerator for possible TargetZones*/
    enum TargetZones {
        A,
        B,
        C
    }

    /* Tested Target Zone Average Values for stack of rings */
    private final double TZAV_0_Reading = 125;   // tested reading for no rings
    private final double TZAV_1_Reading = 110;   // tested reading for one ring
    private final double TZAV_4_Reading = 87;    // tested reading for four rings

    /* Create integer to hold the Target Zone Average Value */
    private int targetZoneAverageValue;

    /* Create enum TargetZones to store the deciphered Target Zone */
    private TargetZones decipheredTargetZone;

    /* Create Elapsed runtimer */
    private ElapsedTime runtime = new ElapsedTime();

    /* Create DcMotors for drive */
    /* We had to make these public to work with Template Autono class */
    public DcMotor FLDrive = null;
    public DcMotor FRDrive = null;
    public DcMotor BLDrive = null;
    public DcMotor BRDrive = null;

    /* Create DcMotors for shooter */
    private DcMotor LShooter = null;
    private DcMotor RShooter = null;

    /* Create other motors and servos */
    public Servo ChickenFinger = null;
    public DcMotor LaChickenWing = null;
    private DcMotor Intake = null;


    /* Create sensors */
    private DistanceSensor FrontDistanceSensor = null; // NEW

    private RevBlinkinLedDriver blinkinLedDriver = null;
    private RevBlinkinLedDriver.BlinkinPattern pattern = null;

    /* Create a webcam */
    OpenCvCamera webcam;

    /* Create Hardware Map */
    HardwareMap hMap =  null;

    public Robot() {
    }



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
        FLDrive.setDirection(DcMotor.Direction.FORWARD);
        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);

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
        LaChickenWing = hardwareMap.get(DcMotor.class, "Wing");
        // Wobble targer servo
        ChickenFinger = hardwareMap.get(Servo.class, "Finger");
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


    /***************************
     *        GETTER           *
     *  get the value of       *
     *  targetZoneAverageValue *
     ***************************/
    public int getTargetZoneAverageValue() {

        return targetZoneAverageValue;

    }


    /***************************
     *        GETTER           *
     *  get the value of       *
     *  desciferedTargetZone   *
     ***************************/
    public TargetZones getDecipheredTargetZone() {

        return decipheredTargetZone;

    }


    /*************************
     *  CarWash - start the  *
     *  rear ring intake     *
     *  carwash spinner      *
     *************************/
    public void CarWash (double speed) {

        Intake.setPower(speed);

    }


    public void StopRobot()
    {
        FLDrive.setPower(0);
        FRDrive.setPower(0);
        BLDrive.setPower(0);
        BRDrive.setPower(0);
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


        double speed = 1;


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

        LaChickenWing.setPower(wingSpeed);

    }


    /***********************************************
     * open or close the servo on the                 *
     * wobble targert lifter                          *
     * Accepts a single argument for se *rvo position *
     ***********************************************/
    public void FingerGrab(double gripPosition) {

        ChickenFinger.setPosition(gripPosition);

    }




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

    // Based largely on EasyOpenCV example WebcamExample
    class RedPipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        Mat processedImage = new Mat();     // Matrix to contain the input image after it is converted to LCrCb colorspace
        Mat processedImageCr = new Mat();   // Matrix to contain just the Cb chanel
        Mat targetZoneSample = new Mat();   // Matrix to contin the image cropped to the area where we expect to find the ring stack

        final int leftMargin = 120;         // Left margin to be cropped off processedImageCr
        final int righMargin = 120;         // Rign margin to be cropped off
        final int topMargin = 50;           // Top margin to be cropped off
        final int botMargin = 140;          // Bottom margin to be cropped off

        final double TZAV_Threshold_A = (TZAV_0_Reading + TZAV_1_Reading) / 2;  // Average of the 0 Ring and 1 Ring values to function as Threshold
        final double TZAV_Threshold_B = (TZAV_1_Reading + TZAV_4_Reading) / 2;  // Average of the 1 Ring and 4 Ring valuse to function as Threshold

        @Override
        public Mat processFrame(Mat input)
        {
            // convert the input RGB image to YCrCb color space
            Imgproc.cvtColor(input, processedImage, Imgproc.COLOR_RGB2YCrCb);
            // extract just the Cb (?) channel to isolate the difference in red
            Core.extractChannel(processedImage, processedImageCr, 2);

            // copy just target regon to a new matrix
            targetZoneSample = processedImageCr.submat(
                    new Rect(
                            new Point(
                                leftMargin,
                                topMargin),
                            new Point(
                            input.cols()-righMargin,
                            input.rows()-botMargin)));

            targetZoneAverageValue = (int) Core.mean(targetZoneSample).val[0];

            if (targetZoneAverageValue > TZAV_Threshold_A) {
                // no rings detected, so Target Zone A is selected
                decipheredTargetZone = TargetZones.A;
            } else if (targetZoneAverageValue > TZAV_Threshold_B) {
                // one ring detected, so Target Zone B is selected
                decipheredTargetZone = TargetZones.B;
            } else{
                // four rings detected, so Target Zone C is selected
                decipheredTargetZone = TargetZones.C;
            }
            idTargetZone(decipheredTargetZone);

            //  Draw a simple box around the area where we expect the target rings to be seen
            Imgproc.rectangle(
                    processedImageCr,
                    new Point(
                            leftMargin,
                            topMargin),
                    new Point(
                            input.cols()-righMargin,
                            input.rows()-botMargin),
                    new Scalar(255, 0, 0), 4);




            return processedImageCr;
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



