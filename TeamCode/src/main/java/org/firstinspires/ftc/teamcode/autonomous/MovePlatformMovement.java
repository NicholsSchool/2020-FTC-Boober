package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Robot;


public class MovePlatformMovement {
    private double driveSpeed = 0.5, turnSpeed = 0.4,
            leftTurn = 90, rightTurn = -leftTurn;
    private double driveTimeOut = 3, turnTimeOut = 2;

    double fastSide = 0.8, slowSide = 0.4;

    private double desiredDistanceFromWall = 14, distanceBackupToPlatform = 6;

    private Function clawDown = new Function() {
        public void execute(){Robot.claw.down();}
        public void stop(){Robot.claw.stop();}
    };

    private Function clawDownWeak = new Function() {
        public void execute(){Robot.claw.move(-0.5);}
        public void stop(){Robot.claw.stop();}
    };


    public void run(boolean isRed)
    {
        Robot.driveTrain.encoderDrive(driveSpeed, -23.5, -23.5, driveTimeOut);
        if(isRed)
            Robot.driveTrain.turnOnHeading(turnSpeed, rightTurn, turnTimeOut);
        else
            Robot.driveTrain.turnOnHeading(turnSpeed, leftTurn, turnTimeOut);

        double distanceToDriveToWall = getDistanceToMoveToWall();
        Robot.driveTrain.encoderDrive(driveSpeed/1.5, distanceToDriveToWall, distanceToDriveToWall, driveTimeOut);

        //Just try once more, we have the time for it.
        distanceToDriveToWall = getDistanceToMoveToWall();
        Robot.driveTrain.encoderDrive(driveSpeed/1.5, distanceToDriveToWall, distanceToDriveToWall, driveTimeOut);

        Robot.driveTrain.turnOnHeading(turnSpeed, 0, turnTimeOut);

        Robot.driveTrain.encoderDrive(driveSpeed/2, -distanceBackupToPlatform, -distanceBackupToPlatform, driveTimeOut);

        Robot.claw.timedMove(false, 2);
        Robot.driveTrain.encoderDrive(driveSpeed, 7, 7, driveTimeOut, clawDown);

        if(isRed)
            Robot.driveTrain.driveAndTurn(fastSide, slowSide, rightTurn, turnTimeOut, clawDownWeak);
        else
            Robot.driveTrain.driveAndTurn(slowSide, fastSide, leftTurn, turnTimeOut, clawDownWeak);

        Robot.claw.timedMove(true, 1.5);
        Robot.driveTrain.encoderDrive(driveSpeed, -5, -5, driveTimeOut);

        if(isRed)
            Robot.driveTrain.turnOnHeading(turnSpeed, rightTurn, turnTimeOut );
        else
            Robot.driveTrain.turnOnHeading(turnSpeed, leftTurn, turnTimeOut );

    }

    public double getDistanceToMoveToWall()
    {
        pause(500);
        double currentDistanceFromWall = Robot.backDistanceSensor.get();
        return  desiredDistanceFromWall - currentDistanceFromWall;
    }

    public void pause(long milliseconds)
    {
        long sleepStart = System.currentTimeMillis();
        while( Robot.opMode.opModeIsActive() && milliseconds > System.currentTimeMillis() - sleepStart)
        { }
    }
}
