package org.firstinspires.ftc.teamcode.roadrunner;


import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;

import java.io.IOException;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name="Custom Test", group = "test")
public class TestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap,telemetry, gamepad1, gamepad2, this);

        RoadRunnerTankDriveOptimized drive = new RoadRunnerTankDriveOptimized();

        Trajectory test2 = null;
        try{
            test2 = CustomTrajectoryLoader.load("Test 2");
            RobotMap.telemetry.addLine("Loaded!");
            RobotMap.telemetry.update();
        }
        catch(IOException ioe)
        {
            RobotMap.telemetry.addLine("Error thrown");
            RobotMap.telemetry.update();
        }
        waitForStart();

        if (isStopRequested()) return;


        drive.followTrajectorySync(
                test2
        );


        sleep(5000);

    }
}
