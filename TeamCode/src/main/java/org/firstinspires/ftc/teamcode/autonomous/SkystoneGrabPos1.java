package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;

@Autonomous(name="Skystone From Position 1", group="Loading Zone Autos")
public class SkystoneGrabPos1 extends LinearOpMode {
    private double driveSpeed = 0.5, turnSpeed = 0.4,
            leftTurn = 90, rightTurn = -leftTurn;
    private double driveTimeOut = 3, turnTimeOut = 2;
    private double skystoneLength = 8;
    private double distanceFromStone = 4.5, distanceAwayFromStone = 5;
    private double secondStoneDriveForward = 30;

    Function clawDown = new Function() {
        public void execute(){Robot.claw.down();}
        public void stop(){Robot.claw.stop();}
    };

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap, FtcDashboard.getInstance().getTelemetry(), gamepad1, gamepad2, this);
        Robot.driveTrain.setBrakeMode(false);
        boolean isRed = Robot.colorPicker.isRed();
        int skyStonePosition = 0;
        while(!isStarted())
            skyStonePosition = Robot.camera.getSkystonePosition(isRed, 1);
        waitForStart();
        long startTime = System.currentTimeMillis();
        run(Robot.colorPicker.isRed(), skyStonePosition);
        RobotMap.telemetry.addData("TIME TAKEN", (System.currentTimeMillis() - startTime)/1000);
        RobotMap.telemetry.update();
    }

    private void run(boolean isRed, int skyStonePos)
    {
        Robot.driveTrain.encoderDrive(driveSpeed,   -23.5, -23.5, 3);

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
        if(skystonePos == 3) {
            Robot.driveTrain.encoderDrive(driveSpeed, 7, 7, driveTimeOut);
            return;
        }

        Robot.driveTrain.encoderDrive(0.8, secondStoneDriveForward, secondStoneDriveForward, driveTimeOut);

        getSecondStone(isRed, skystonePos);

        Robot.claw.timedMove(true, 2);
        Robot.driveTrain.encoderDrive(driveSpeed,7,7,driveTimeOut);
    }

    private void getFirstStone(boolean isRed, int skystonePos)
    {
        pause(500);
        double distanceForFirstStone = getDistanceFromWall(skystonePos, true);

        Robot.driveTrain.encoderDrive(driveSpeed, distanceForFirstStone, distanceForFirstStone,driveTimeOut);

        Robot.driveTrain.turnOnHeading(turnSpeed, 0, turnTimeOut);

        pause(500);
        double extraDistance = Robot.backDistanceSensor.get() - distanceFromStone;

        Robot.driveTrain.encoderDrive(0.25, -extraDistance, -extraDistance, driveTimeOut);

        Robot.claw.timedMove(false, 1);

        Robot.driveTrain.encoderDrive(driveSpeed, distanceAwayFromStone, distanceAwayFromStone, driveTimeOut, clawDown);

        if(isRed)
            Robot.driveTrain.turnOnHeading(turnSpeed, rightTurn, turnTimeOut, clawDown);
        else
            Robot.driveTrain.turnOnHeading(turnSpeed, leftTurn, turnTimeOut, clawDown);


        Robot.driveTrain.encoderDrive(1.0, -(52 - distanceForFirstStone), -(52 - distanceForFirstStone), driveTimeOut);
    }

    private void getSecondStone(boolean isRed, int skystonePos)
    {

        double distanceForSecondStone = getDistanceFromWall(skystonePos, false);
        Robot.driveTrain.encoderDrive(driveSpeed,  distanceForSecondStone, distanceForSecondStone, driveTimeOut);

        Robot.driveTrain.turnOnHeading(turnSpeed, 0, turnTimeOut);

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

    private double getDistanceFromWall(int skystonePos, boolean isFirst)
    {
        double turnGap = 3;
        if(skystonePos == 2)
            turnGap = 3.5;

        int numSkystones = 6;
        if(!isFirst)
            numSkystones = 3;

        double desiredDistanceFromWall = (numSkystones - skystonePos) * skystoneLength - turnGap;
        double currentDistanceFromWall;
        pause(500);
        if(isFirst)
            currentDistanceFromWall = Robot.backDistanceSensor.get();
        else
            currentDistanceFromWall = Robot.frontDistanceSensor.get();

        RobotMap.telemetry.addData("Distance from Wall As Detected", currentDistanceFromWall);
        RobotMap.telemetry.addData("The Desired Distance From Wall", desiredDistanceFromWall);
        RobotMap.telemetry.update();
      //  pauseTillButtonPressed();
        return desiredDistanceFromWall - currentDistanceFromWall;
    }

    public void pause(long milliseconds)
    {
        long sleepStart = System.currentTimeMillis();
        while( opModeIsActive() && milliseconds > System.currentTimeMillis() - sleepStart)
        { }
    }

    //ONLY FOR TESTING!!!!
    public void pauseTillButtonPressed()
    {
        while(opModeIsActive() && !RobotMap.g1.a)
        {}
    }
}
