package org.firstinspires.ftc.teamcode.autonomous.movements;

import org.firstinspires.ftc.teamcode.autonomous.Function;
import org.firstinspires.ftc.teamcode.util.Robot;

/**
 * A "Movement" is the general path the robot needs to take to accomplish the task.
 * It has adjustable values so that certain starting, ending, and speeds can be configured
 *
 * This class handles moving the platform.
 */
public class MovePlatformMovement {
    private double driveSpeed = 1, turnSpeed = 0.4,
            leftTurn = 90, rightTurn = -leftTurn;

    private double pullSpeed = 0.5;
    private double driveTimeOut = 3, turnTimeOut = 2;

    private boolean runGoBack = true;
    private double fastSide = 0.8, slowSide = 0.4;

    private double desiredDistanceFromWall = 14, distanceBackupToPlatform = 6;

    public void setRunGoBack(boolean goBack)
    {
        runGoBack = goBack;
    }

    public void setDistanceBackupToPlatform(double distance)
    {
        distanceBackupToPlatform = distance;
    }

    private Function clawDown = new Function() {
        public void execute(){Robot.claw.down();}
        public void stop(){Robot.claw.stop();}
    };

    private Function clawDownWeak = new Function() {
        public void execute(){Robot.claw.move(-0.5);}
        public void stop(){Robot.claw.stop();}
    };

    private Function clawUpWeak = new Function() {
        public void execute(){Robot.claw.move(0.2);}
        public void stop(){Robot.claw.stop();}
    };


    public void run(boolean isRed)
    {

        Robot.driveTrain.encoderDrive(driveSpeed, -23.5, -23.5, driveTimeOut);
        if(isRed)
            Robot.driveTrain.turnOnHeading(turnSpeed, rightTurn, turnTimeOut, clawUpWeak);
        else
            Robot.driveTrain.turnOnHeading(turnSpeed, leftTurn, turnTimeOut, clawUpWeak);

        double distanceToDriveToWall = getDistanceToMoveToWall();
        Robot.driveTrain.encoderDrive(driveSpeed/1.5, distanceToDriveToWall, distanceToDriveToWall, driveTimeOut);

        //Just try once more, we have the time for it.
        distanceToDriveToWall = getDistanceToMoveToWall();
        Robot.driveTrain.encoderDrive(driveSpeed/1.5, distanceToDriveToWall, distanceToDriveToWall, driveTimeOut);

        /************************************************************************/

        grabPlatform(isRed);

    }

    public void grabPlatform(boolean isRed)
    {
        Robot.driveTrain.turnOnHeading(turnSpeed, 0, turnTimeOut);

        Robot.driveTrain.encoderDrive(pullSpeed, -distanceBackupToPlatform, -distanceBackupToPlatform, driveTimeOut);

        /************************************************************************/

        Robot.claw.timedMove(false, 2);
        Robot.driveTrain.encoderDrive(pullSpeed, 7, 7, driveTimeOut, clawDown);

        if(isRed)
            Robot.driveTrain.driveAndTurn(fastSide, slowSide, rightTurn, turnTimeOut, clawDownWeak);
        else
            Robot.driveTrain.driveAndTurn(slowSide, fastSide, leftTurn, turnTimeOut, clawDownWeak);

        /************************************************************************/

        Robot.claw.timedMove(true, 1.5);

        double moveBack = 5;
        if(Math.abs(Robot.gyro.getHeading()) > 60 )
            moveBack = 10;

        //Issue where the claw servo latches on to platform on red side. So we don't want to do this
        if(!isRed && runGoBack)
            Robot.driveTrain.encoderDrive(pullSpeed, -moveBack, -moveBack, driveTimeOut);

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
