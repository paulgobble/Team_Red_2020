package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class driverDisplay {

    //private String sayThis = "Hello world";

    public static Telemetry.Item transmit() {

        Telemetry.Item teleMeItem = telemetry.addData("Say What?", "Hello world");

        return teleMeItem;

    }

}
