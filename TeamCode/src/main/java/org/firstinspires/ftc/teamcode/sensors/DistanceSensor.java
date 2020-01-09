package org.firstinspires.ftc.teamcode.sensors;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.RobotMap;

public class DistanceSensor {

    public double get()
    {
       return RobotMap.distanceSensor.getDistance(DistanceUnit.INCH);
    }

    public void print()
    {
        RobotMap.telemetry.addData("Distance", get());
    }
}
