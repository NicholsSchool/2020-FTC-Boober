package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryLoader;

import java.io.File;
import java.io.IOException;

public class CustomTrajectoryLoader {

    public static Trajectory load(String name) throws IOException
    {
        return TrajectoryLoader.load(new File("TeamCode/src/main/assets/trajectory/Test 2.yaml"));
    }
}
