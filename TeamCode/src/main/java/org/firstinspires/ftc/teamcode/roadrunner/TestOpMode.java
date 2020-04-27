package org.firstinspires.ftc.teamcode.roadrunner;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.autonomous.Function;
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
    private Function clawDown = new Function() {
        public void execute(){Robot.claw.down();
            RobotMap.telemetry.addLine("Running Claw Down Function");
            RobotMap.telemetry.update();
        }
        public void stop(){Robot.claw.stop();}
    };

    private Pose2d startPose = new Pose2d(-40, 64, Math.toRadians(90));
    private RoadRunnerTankDriveOptimized drive;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap,telemetry, gamepad1, gamepad2, this);

        drive = new RoadRunnerTankDriveOptimized(hardwareMap);
        drive.setPoseEstimate(startPose);

        int skystonePos = 2;
        waitForStart();

        if (isStopRequested()) return;

        getStoneOne(skystonePos);
        Robot.claw.timedMove(true, 1.5);

        if(skystonePos != 3)
            getStoneTwo(skystonePos);

        RobotMap.telemetry.addLine("Finished Path");
        RobotMap.telemetry.update();

        pause(5000);

    }


    public void getStoneOne(int skystonePos)
    {
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(true)
                        .splineTo(new Pose2d(-29 - (8 * (skystonePos - 1)), 34, Math.toRadians(90)))
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

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(true)
                        .splineTo(new Pose2d(-10, 37, Math.toRadians(180)))
                        .splineTo(new Pose2d(18, 37, Math.toRadians(180)))
                        .build(),
                clawDown
        );
    }

    public void getStoneTwo(int skystonePos)
    {

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(false)

                        .splineTo(new Pose2d(-35, 42, Math.toRadians(180)))
                        .splineTo(new Pose2d(-52  - (8 * (skystonePos - 1)) , 38, Math.toRadians(90)))
                        .build()
        );

        double turnError = Math.toRadians(90) - drive.getPoseEstimate().getHeading();
        drive.turnSync(turnError);

        double extraDistance = getDesiredDistanceFromBlock(distanceFromStone - 0.1);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(extraDistance < 0)
                        .forward(extraDistance)
                        .build()
        );

        Robot.claw.timedMove(false, 1.5);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(false)
                        .splineTo(new Pose2d(-58, 48, Math.toRadians(180)))
                        .setReversed(true)
                        .splineTo(new Pose2d(-24, 48, Math.toRadians(180)))
                        .splineTo(new Pose2d(18, 37, Math.toRadians(180)))
                        .build(), clawDown
        );
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
