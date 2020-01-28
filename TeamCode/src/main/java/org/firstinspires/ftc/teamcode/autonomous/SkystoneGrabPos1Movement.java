package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;


public class SkystoneGrabPos1Movement {
    private double driveSpeed = 1, turnSpeed = 0.4,
            leftTurn = 90, rightTurn = -leftTurn;
    private double driveTimeOut = 3, turnTimeOut = 2;
    private double skystoneLength = 8;
    private double distanceFromStone = 3.5, distanceAwayFromStone = 5;
    private double secondStoneDriveForward = 30;
    private boolean park = true;

    Function clawDown = new Function() {
        public void execute(){Robot.claw.down();}
        public void stop(){Robot.claw.stop();}
    };

    public void setPark(boolean park)
    {
        this.park = park;
    }

    public void setDistanceAwayFromStone(double distance)
    {
        distanceAwayFromStone = distance;
    }

    public void runMovement() throws InterruptedException {

        Robot.driveTrain.setBrakeMode(true);
        boolean isRed = Robot.colorPicker.isRed();
        int skyStonePosition = 0;
        while(!Robot.opMode.isStarted()) {
            skyStonePosition = Robot.camera.getSkystonePosition(isRed, 1);
            RobotMap.telemetry.addData("Skystone Position", skyStonePosition);
            RobotMap.telemetry.update();
        }
        Robot.opMode.waitForStart();
        long startTime = System.currentTimeMillis();
        run(Robot.colorPicker.isRed(), skyStonePosition);
        RobotMap.telemetry.addData("TIME TAKEN", (System.currentTimeMillis() - startTime)/1000);
        Robot.driveTrain.printInfo();
        RobotMap.telemetry.update();
    }

    private void run(boolean isRed, int skyStonePos)
    {
        if(skyStonePos == 1)
            driveSpeed = 0.5;
        Robot.driveTrain.encoderDrive(driveSpeed,   -24, -24, 3);

        getStone(isRed, skyStonePos );
    }



    private void getStone(boolean isRed, int skystonePos )
    {
        if(isRed)
            Robot.driveTrain.turnOnHeading(turnSpeed, leftTurn, turnTimeOut);
        else
            Robot.driveTrain.turnOnHeading(turnSpeed, rightTurn, turnTimeOut);

        getFirstStone(isRed, skystonePos);

        //claw up
        Robot.claw.timedMove(true, 2);
        if(isRed)
            Robot.driveTrain.turnOnHeading(turnSpeed,rightTurn, turnTimeOut);
        else
            Robot.driveTrain.turnOnHeading(turnSpeed,leftTurn,turnTimeOut);


        //For third case, just park because we can't reach the end of the field.
        if(skystonePos > 0) {
            if(park)
                Robot.driveTrain.encoderDrive(driveSpeed, 9, 9, driveTimeOut);
            return;
        }

        Robot.driveTrain.encoderDrive(0.8, secondStoneDriveForward, secondStoneDriveForward, driveTimeOut);

        getSecondStone(isRed, skystonePos);

        Robot.claw.timedMove(true, 2);
        Robot.driveTrain.encoderDrive(driveSpeed,18,18,driveTimeOut);
    }

    private void getFirstStone(boolean isRed, int skystonePos)
    {
        double distanceForFirstStone = getDistanceFromWall(skystonePos, true);

        Robot.driveTrain.encoderDrive(driveSpeed, distanceForFirstStone, distanceForFirstStone,driveTimeOut);


        //Do it once more because we aren't going for second stone and have the time to adjust.
        distanceForFirstStone = getDistanceFromWall(skystonePos, true);
        Robot.driveTrain.encoderDrive(driveSpeed, distanceForFirstStone, distanceForFirstStone,driveTimeOut);

        Robot.driveTrain.turnOnHeading(turnSpeed, 0, turnTimeOut);

        pause(500);
        double extraDistance = Robot.backDistanceSensor.get() - distanceFromStone;
        Robot.driveTrain.encoderDrive(0.25, - extraDistance, -extraDistance, driveTimeOut);

        pause(500);
        extraDistance = Robot.backDistanceSensor.get() - distanceFromStone;
        Robot.driveTrain.encoderDrive(0.25, - extraDistance, -extraDistance, driveTimeOut);

        Robot.claw.timedMove(false, 1);

        Robot.driveTrain.encoderDrive(driveSpeed/3, distanceAwayFromStone, distanceAwayFromStone, driveTimeOut, clawDown);

        if(isRed)
            Robot.driveTrain.turnOnHeading(turnSpeed, rightTurn, turnTimeOut, clawDown);
        else
            Robot.driveTrain.turnOnHeading(turnSpeed, leftTurn, turnTimeOut, clawDown);

        double currentPosition = getDesiredDistanceFromWall(skystonePos, true);
        Robot.driveTrain.encoderDrive(1.0, -(80 - currentPosition), -(80 - currentPosition), driveTimeOut);
    }

    private void getSecondStone(boolean isRed, int skystonePos)
    {

        double distanceForSecondStone = getDistanceFromWall(skystonePos, false);
        Robot.driveTrain.encoderDrive(driveSpeed,  distanceForSecondStone, distanceForSecondStone, driveTimeOut);

        Robot.driveTrain.turnOnHeading(turnSpeed, 0, turnTimeOut);

        pause(500);
        double extraDistance = Robot.backDistanceSensor.get() - distanceFromStone;
        Robot.driveTrain.encoderDrive(0.25, -extraDistance, -extraDistance, driveTimeOut);
        //claw down
        Robot.claw.timedMove(false, 1);

        Robot.driveTrain.encoderDrive(driveSpeed, distanceAwayFromStone, distanceAwayFromStone, driveTimeOut);
        if(isRed)
            Robot.driveTrain.turnOnHeading(turnSpeed, rightTurn, turnTimeOut, clawDown);
        else
            Robot.driveTrain.turnOnHeading(turnSpeed, leftTurn, turnTimeOut, clawDown);

        Robot.driveTrain.encoderDrive(1.0, -distanceForSecondStone - secondStoneDriveForward, -distanceForSecondStone - secondStoneDriveForward, driveTimeOut);
        
    }

    private double getDesiredDistanceFromWall(int skystonePos, boolean isFirst)
    {
        double turnGap = 4.5;

        int numSkystones = 6;
        if(!isFirst)
            numSkystones = 3;

        return (numSkystones - skystonePos) * skystoneLength - turnGap;
    }

    private double getDistanceFromWall(int skystonePos, boolean isFirst)
    {

        double desiredDistanceFromWall =getDesiredDistanceFromWall(skystonePos, isFirst);
        double currentDistanceFromWall;
        pause(500);
        if(isFirst)
            currentDistanceFromWall = Robot.backDistanceSensor.get();
        else
            currentDistanceFromWall = Robot.frontDistanceSensor.get();

        RobotMap.telemetry.addData("Distance from Wall As Detected", currentDistanceFromWall);
        RobotMap.telemetry.addData("The Desired Distance From Wall", desiredDistanceFromWall);
        RobotMap.telemetry.addData("Distance going to drive", (desiredDistanceFromWall - currentDistanceFromWall));
        RobotMap.telemetry.update();
        //   pauseTillButtonPressed();
        return desiredDistanceFromWall - currentDistanceFromWall;
    }

    public void pause(long milliseconds)
    {
        long sleepStart = System.currentTimeMillis();
        while( Robot.opMode.opModeIsActive() && milliseconds > System.currentTimeMillis() - sleepStart)
        { }
    }

}
