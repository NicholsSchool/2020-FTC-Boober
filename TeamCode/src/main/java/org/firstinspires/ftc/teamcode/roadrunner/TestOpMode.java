package org.firstinspires.ftc.teamcode.roadrunner;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;

import java.io.IOException;
import kotlin.Unit;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name="Custom Test", group = "test")
public class TestOpMode extends LinearOpMode {

    private double distanceFromStone = 3.6;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap,telemetry, gamepad1, gamepad2, this);

        RoadRunnerTankDriveOptimized drive = new RoadRunnerTankDriveOptimized(hardwareMap);


        drive.setPoseEstimate(new Pose2d(-40, 64, Math.toRadians(90)));
        RobotMap.telemetry.setAutoClear(false);
        RobotMap.telemetry.addLine("Waiting for start");
        RobotMap.telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        RobotMap.telemetry.addLine("Starting");
        RobotMap.telemetry.update();

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .setReversed(true)
                .splineTo(new Pose2d(-29, 34, Math.toRadians(90)))
                .build()
        );


        double extraDistance = getDesiredDistanceFromBlock(distanceFromStone);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .setReversed(extraDistance < 0)
                .forward(extraDistance)
                .build()
        );


        Robot.claw.timedMove(false, 1.5);
        RobotMap.telemetry.addLine("Arm Down!");
        RobotMap.telemetry.addLine(drive.getPoseEstimate().toString());
        RobotMap.telemetry.update();
        drive.followTrajectorySync(
                 drive.trajectoryBuilder()
                .setReversed(true)
                .splineTo(new Pose2d(-10, 37, Math.toRadians(180)))
                .splineTo(new Pose2d(18, 37, Math.toRadians(180)))
                .build()
        );

        Robot.claw.timedMove(true, 1.5);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(false)

                        .splineTo(new Pose2d(-35, 42, Math.toRadians(180)))
                        .splineTo(new Pose2d(-52, 38, Math.toRadians(90)))
                        .build()
        );
        double turnError = Math.toRadians(90) - drive.getPoseEstimate().getHeading();
        drive.turnSync(turnError);

        extraDistance = getDesiredDistanceFromBlock(distanceFromStone - 0.1);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(extraDistance < 0)
                        .forward(extraDistance)
                        .build()
        );

        Robot.claw.timedMove(false, 1.5);
        RobotMap.telemetry.addLine("Arm Down!");
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(false)
                        .splineTo(new Pose2d(-52, 43, Math.toRadians(90)))
                        .setReversed(true)
                        .splineTo(new Pose2d(-10, 37, Math.toRadians(180)))
                        .splineTo(new Pose2d(18, 37, Math.toRadians(180)))
                        .build()
        );

        RobotMap.telemetry.addLine(drive.getPoseEstimate().toString());
        RobotMap.telemetry.update();

        RobotMap.telemetry.addLine("Finished Path");
        RobotMap.telemetry.update();

        drive.updatePoseEstimate();

        pause(5000);

    }

    public double getDesiredDistanceFromBlock(double distanceFromStone)
    {
        pause(500);
        double currentDistanceAway = Robot.backDistanceSensor.get();
        telemetry.addData("Back distance Sensor", currentDistanceAway);
        double extraDistance = distanceFromStone - currentDistanceAway;
        telemetry.addData("Extra distance",extraDistance);
        telemetry.update();
        return extraDistance;
    }

    public void pause(long milliseconds)
    {
        long sleepStart = System.currentTimeMillis();
        while( Robot.opMode.opModeIsActive() && milliseconds > System.currentTimeMillis() - sleepStart)
        { }
    }
}

/*
* - x: -40.0
  y: 64.0
  heading: 1.5707963267948966
- x: -32.0
  y: 30.0
  heading: 1.5707963267948966
- x: -10.0
  y: 33.0
  heading: 3.141592653589793
- x: 18.0
  y: 33.0
  heading: 3.141592653589793
  * */
