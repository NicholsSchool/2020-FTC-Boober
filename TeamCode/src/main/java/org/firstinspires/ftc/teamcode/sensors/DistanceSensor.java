package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.RobotMap;

public class DistanceSensor {

    private Rev2mDistanceSensor distanceSensor;
    private String name;
    public DistanceSensor(Rev2mDistanceSensor distanceSensor, String name)
    {
        this.distanceSensor = distanceSensor;
        this.name = name;
    }

    public double get()
    {
       return distanceSensor.getDistance(DistanceUnit.INCH);
    }

    public void print()
    {
        RobotMap.telemetry.addData(name + " Distance", get());
    }
}
