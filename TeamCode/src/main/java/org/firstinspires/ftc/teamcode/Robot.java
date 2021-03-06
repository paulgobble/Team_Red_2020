// Version 1.9.9

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

import java.util.ArrayList;
import java.util.Collections;

public class Robot {

    /* Establish Essential Robot Modes */
    private int driveDirectionModifyer = 1;     // What do we consider "Forward"

    private boolean forceFieldArmed = false;    // Has the force field system been activated by pilot
    private boolean inDangerZone = false;       // Is the robot within the stop distance limit
    private boolean forceFieldTriggered = false;  // Has the Robot turned on the force field 'cause it wsa activated by pilot and robot is within the stop distance
    private final double stopDistance = 14.0;

    private boolean streamingVideo;             // is the connection to the webcam open

    private boolean allowVideoScan = false;           // is it OK to begin scanning the video pipeline

    private boolean scanVideoCompleted = false;      // Has the video pipeline completed one full scan and captured TAZVs

    private double scanCompleteTime;               // store the length of time in seconds it took to complete the video scan

    /* Define Enumerator for possible TargetZones*/
    enum TargetZones {
        A,  //no rings
        B,  // one ring
        C,  // four rings
        S,  // scanning
        X   // pre-scan
    }

    /* Tested Target Zone Average Values for stack of rings */
    private final double TZAV_0_Reading = 114;   // tested reading for no rings - was 125
    private final double TZAV_1_Reading = 112;   // tested reading for one ring - was 110
    private final double TZAV_4_Reading = 95;    // tested reading for four rings - was 87

    /* Create an array to hold the sorted TZAVs */
    //private ArrayList<Integer> TZAVs_Array;

    /* Create integer to hold the Target Zone Average Value */
    private int targetZoneAverageValue;

    /* Create enum TargetZones to store the deciphered Target Zone */
    private TargetZones decipheredTargetZone;

    /* Create Elapsed runtimer */
    private ElapsedTime runtime = new ElapsedTime();

    /* Create DcMotors for drive */
    /* We had to make these public to work with Template Autono class */
    public DcMotorEx FLDrive = null;
    public DcMotorEx FRDrive = null;
    public DcMotorEx BLDrive = null;
    public DcMotorEx BRDrive = null;

    /* Create DcMotors for shooter */
    public DcMotor LShooter = null;
    public DcMotor RShooter = null;

    /* Create other motors and servos */
    public Servo ChickenFinger = null;
    public DcMotor LaChickenWing = null;
    public DcMotor Intake = null;


    /* Create sensors */
    private DistanceSensor FrontDistanceSensor = null;

    private ColorSensor FrontRightColorSensor = null;

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
        FLDrive = hardwareMap.get(DcMotorEx.class, "FLDrive");
        FRDrive = hardwareMap.get(DcMotorEx.class, "FRDrive");
        BRDrive = hardwareMap.get(DcMotorEx.class, "BRDrive");
        BLDrive = hardwareMap.get(DcMotorEx.class, "BLDrive");
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
        LaChickenWing.setDirection(DcMotorSimple.Direction.FORWARD);
        // Wobble targer servo
        ChickenFinger = hardwareMap.get(Servo.class, "Finger");
        //Lift.setDirection(DcMotor.Direction.FORWARD);

        // Sensors
        FrontDistanceSensor = hardwareMap.get(DistanceSensor.class,"FrontDistanceSensor");
        FrontRightColorSensor = hardwareMap.get(ColorSensor.class, "frColor");
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


    /*****************************
     *                           *
     *    Getters and Setters    *
     *                           *
     *****************************/

    // SETTER - Toggle the value of forwardDriveMode
    public void setForwardDriveMode () {
        if (driveDirectionModifyer == 1) {
            driveDirectionModifyer = -1;
        } else {
            driveDirectionModifyer = 1;
        }
    }
    // Getter - report the valuse of driveDirectionModifyer
    public String getDirectionMode () {
        if (driveDirectionModifyer == 1) {
            return "FORWARD (Shooter)";
        } else {
            return "REVERSE (Wedge)";
        }
    }
    // Setter - set the valuse of forceFieldOn
    public void toggleForceField() {

        if (forceFieldArmed) {
            forceFieldArmed = false;
        } else {
            forceFieldArmed = true;
        }
    }
    // Getter - report the valuse of forceFieldOn
    public String isForceFieldOn() {

        if (forceFieldTriggered) {
            return "ON";
        } else if (forceFieldArmed) {
            return "Armed";
        } else {
            return "OFF";
        }
    }
    // Get the value of FrontDistanceSensor
    public double getFrontDistance () {

        return FrontDistanceSensor.getDistance(DistanceUnit.INCH);

    }
    // Setter - set the value of streamingVideo
    public void setStreamingVideo(boolean onOrOff){

        streamingVideo = onOrOff;

        //if(!onOrOff) webcam.closeCameraDevice();
    }
    // Getter - report the value of targetZoneAverageValue
    public int getTargetZoneAverageValue() {

        return targetZoneAverageValue;

    }
    // Getter - report the value of decipheredTargetZone
    public TargetZones getDecipheredTargetZone() {

        return decipheredTargetZone;

    }
    // Getter - report the valure of scanCompleteTime
    public double getScanCompleteTime() {

        return scanCompleteTime;
    }
    // Setter - receive the OK to start processing the video pipeline
    public void startScanning(boolean startScan){

        allowVideoScan = startScan;

    }
    // Getter - return the alpha value of the floor facing color sensor
    public double getFRColor_alpha(){

        return FrontRightColorSensor.alpha();

    }


    public void StopRobot()
    {
        FLDrive.setPower(0);
        FRDrive.setPower(0);
        BLDrive.setPower(0);
        BRDrive.setPower(0);
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

        if ((forceFieldArmed) && (forceFieldTriggered)) {
            driveAdjust = 0;
            turnAdjust = 0;
            strafeAdjust = masterPowerLimit / 2;
        } else {
            driveAdjust = masterPowerLimit;
            turnAdjust = masterPowerLimit;
            strafeAdjust = masterPowerLimit;
        }

        FRPower = ((drive * driveAdjust * driveDirectionModifyer) + (strafe * strafeAdjust * driveDirectionModifyer) + (turn * turnAdjust) ) ;
        FLPower = ((drive * driveAdjust * driveDirectionModifyer) - (strafe * strafeAdjust * driveDirectionModifyer) - (turn * turnAdjust) ) ;
        BRPower = ((drive * driveAdjust * driveDirectionModifyer) - (strafe * strafeAdjust * driveDirectionModifyer) + (turn * turnAdjust) ) ;
        BLPower = ((drive * driveAdjust * driveDirectionModifyer) + (strafe * strafeAdjust * driveDirectionModifyer) - (turn * turnAdjust) ) ;

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


    /******************************************
     * Raise or lower La Chicke WIng          *
     * wobble targert lifter                  *
     * Accepts a argument for DC motor spoeed *
     ******************************************/
    public void FlapWing(double wingSpeed) {

        LaChickenWing.setPower(wingSpeed);

    }


    /************************************************
     * open or close the servo on the               *
     * wobble targert lifter                        *
     * Accepts a single argument for servo position *
     ************************************************/
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
            case S:
                pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE;
                break;
            case X:
                pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
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
        Mat scanZoneSample = new Mat();   // Matrix to contin the image cropped to the area where we expect to find the ring stack

        final int leftMargin = 90;         // Left margin to be cropped off processedImageCr
        final int righMargin = 170;         // Rign margin to be cropped off
        final int topMargin = 60;           // Top margin to be cropped off
        final int botMargin = 130;          // Bottom margin to be cropped off

        final int leftScanPadding = 0;     // Left padding of frame not to be scanned
        final int rightScanPadding = 0;    // Right padding of frame not to be scanned
        final int topScanPadding = 10;      // Top padding of frame not to be scanned
        final int botScanPadding = 120;      // Bottome padding of frame not to be scanned

        final  int scanStep = 5;

        final double TZAV_Threshold_A = (TZAV_0_Reading + TZAV_1_Reading) / 2;  // Average of the 0 Ring and 1 Ring values to function as Threshold
        final double TZAV_Threshold_B = (TZAV_1_Reading + TZAV_4_Reading) / 2;  // Average of the 1 Ring and 4 Ring values to function as Threshold

        ArrayList<Integer> these_TZAVs = new ArrayList<Integer>(); // Create an ArrayList object

        @Override
        public Mat processFrame(Mat input)
        {
            // convert the input RGB image to YCrCb color space
            Imgproc.cvtColor(input, processedImage, Imgproc.COLOR_RGB2YCrCb);
            // extract just the Cb (?) channel to isolate the difference in red
            Core.extractChannel(processedImage, processedImageCr, 2);

            // Compute the ideal center scan zone rectangle
            final double frameWidth = input.cols();
            final double frameHeight = input.rows();
            Point upperLeft = new Point(leftMargin, topMargin);
            Point lowerRight = new Point(frameWidth - righMargin, frameHeight - botMargin);
            Rect centerScanRect = new Rect(upperLeft, lowerRight);

            int zoneWidth = centerScanRect.width;
            int zoneHeight = centerScanRect.height;

            ElapsedTime scanTimer = new ElapsedTime();

            if(allowVideoScan && !scanVideoCompleted) {
                scanTimer.reset();

                //idTargetZone(TargetZones.S);

                // loop through columns
                for (int thisX = leftScanPadding; thisX < frameWidth - zoneWidth - rightScanPadding; thisX = thisX + scanStep) {

                    for (int thisY = topScanPadding; thisY < frameHeight - zoneHeight - botScanPadding; thisY = thisY + scanStep) {

                        // copy just target regon to a new matrix
                        scanZoneSample = processedImageCr.submat(new Rect(thisX, thisY, zoneWidth, zoneHeight));
                        // convert the matrix single color channel averaged numeric value and add this value to an arrayList
                        these_TZAVs.add((int) Core.mean(scanZoneSample).val[0]);

                    }

                }
                // capture the length of time it took to complete the scan
                scanCompleteTime = scanTimer.seconds();

                // Sort the arrayList of TZAVs so the greatest int member is in location 0
                Collections.sort(these_TZAVs);

                //TZAVs_Array = these_TZAVs;
                targetZoneAverageValue = these_TZAVs.get(0);

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

                //scanVideoCompleted = true;  // commented out for debugging

            } // end if(streamingVideo)


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

            //  Draw a second simple box around the area to be scanned
            Imgproc.rectangle(
                    processedImageCr,
                    new Point(
                            leftScanPadding,
                            topScanPadding),
                    new Point(
                            input.cols()-rightScanPadding,
                            input.rows()-botScanPadding),
                    new Scalar(255, 0, 0), 4);

            return processedImageCr;

        } // end processFrame

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
        } // end onViewportTapped

    } // end nested Class RedPipeline



} // end Class Robot