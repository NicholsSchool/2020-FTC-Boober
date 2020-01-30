package org.firstinspires.ftc.teamcode.autonomous.movements;

import org.firstinspires.ftc.teamcode.util.Robot;

/**
 * A "Movement" is the general path the robot needs to take to accomplish the task.
 * It has adjustable values so that certain starting, ending, and speeds can be configured
 *
 * This class handles parking
 */
public class ParkMovement {
    private double driveSpeed = 0.5, turnSpeed = 0.4,
            leftTurn = 90, rightTurn = -leftTurn;
    private double driveTimeOut = 3, turnTimeOut = 2;

    private double backUpDistance = 2;
    private double moveDistance = 10;

    public void runMovement(boolean isRed, boolean isLoadingZone, boolean endFar)
    {
        if(!isLoadingZone)
            isRed = !isRed;

        if(endFar)
            backUpDistance = 25;

        Robot.driveTrain.encoderDrive(driveSpeed, -backUpDistance, -backUpDistance, driveTimeOut);

        if(isRed)
            Robot.driveTrain.turnOnHeading(turnSpeed, leftTurn, turnTimeOut);
        else
            Robot.driveTrain.turnOnHeading(turnSpeed, rightTurn, turnTimeOut);

        Robot.driveTrain.encoderDrive(driveSpeed,moveDistance, moveDistance, driveTimeOut );
    }
}
