package org.firstinspires.ftc.teamcode.autonomous.movements;

import org.firstinspires.ftc.teamcode.util.Robot;

public class ParkMovement {
    private double driveSpeed = 1, turnSpeed = 0.4,
            leftTurn = 90, rightTurn = -leftTurn;
    private double driveTimeOut = 3, turnTimeOut = 2;

    private double backUpDistance = 2;
    private double moveDistance = 3;

    public void runMovement(boolean isRed, boolean isLoadingZone, boolean endFar)
    {
        if(!isLoadingZone)
            isRed = !isRed;

        if(endFar)
            backUpDistance = 30;

        Robot.driveTrain.encoderDrive(driveSpeed, -backUpDistance, -backUpDistance, driveTimeOut);

        if(isRed)
            Robot.driveTrain.turnOnHeading(turnSpeed, leftTurn, turnTimeOut);
        else
            Robot.driveTrain.turnOnHeading(turnSpeed, leftTurn, turnTimeOut);

        Robot.driveTrain.encoderDrive(driveSpeed,moveDistance, moveDistance, driveTimeOut );
    }
}
