package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name="pidfTuner", group="Autonomous")
public class pdifTuner extends LinearOpMode {

    Robot robot = new Robot();

    double currentVelocity;
    double maxVelocity = 0.0;

    double testTime = 5;

    private final ElapsedTime testTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        //robot.FLDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //robot.FRDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //robot.BLDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //robot.BRDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        testTimer.reset();

        //robot.FLDrive.setPower(1);

        while (opModeIsActive() && (testTimer.seconds() < testTime)) {
            currentVelocity = robot.FLDrive.getVelocity();

            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }

            telemetry.addData("Motor", "FLDrive");
            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();
        }
    }



}
