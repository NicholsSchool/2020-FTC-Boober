package org.firstinspires.ftc.teamcode.autonomous.movements;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;

public class FaceBuildZone {

    double driveSpeed, turnSpeed, leftTurn, rightTurn;
    public FaceBuildZone(double driveSpeed, double turnSpeed, double leftTurn, double rightTurn)
    {
        this.driveSpeed = driveSpeed;
        this.turnSpeed = turnSpeed;
        this.leftTurn = leftTurn;
        this.rightTurn = rightTurn;
    }

    public void run(boolean isRed)
    {
        Robot.driveTrain.halfEncoderDrive(driveSpeed, 12, 12);
        RobotMap.telemetry.addLine("About to turn");
        RobotMap.telemetry.update();
        if(isRed)
            Robot.driveTrain.turn(turnSpeed, rightTurn, 3);
        else
            Robot.driveTrain.turn(turnSpeed, leftTurn, 3);
    }
}
