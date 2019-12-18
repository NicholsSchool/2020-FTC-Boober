package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous(name="Skystone From Position 1", group="Loading Zone Autos")
public class SkystoneGrabPos1 extends LinearOpMode {
    private double driveSpeed = 0.5, turnSpeed = 0.4,
            leftTurn = 90, rightTurn = -leftTurn;
    private double driveTimeOut = 3, turnTimeOut = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap, telemetry, gamepad1, gamepad2, this);
        Robot.driveTrain.setBrakeMode(false);
        boolean isRed = Robot.colorPicker.isRed();
        int skyStonePosition = Constants.TEST_SKYSTONE_POSITION;
//        while(!isStarted())
//            skyStonePosition = Robot.camera.getSkystonePosition(isRed, 1);
        waitForStart();
        run(Robot.colorPicker.isRed(), skyStonePosition);
    }

    private void run(boolean isRed, int skyStonePos)
    {
        Robot.driveTrain.encoderDrive(driveSpeed, -20.5, -20.5, 3);

        switch(skyStonePos)
        {
            default:
            case 1:
                newGetStone(isRed, 13.5, 62.5);
                break;
            case 2:
                newGetStone(isRed, 5.5, 68.5);
                break;
            case 3:
                newGetStone(isRed, -3.5, 0);
        }
    }



    private void newGetStone(boolean isRed, double distanceForFirstStone, double distanceForSecondStone)
    {
        if(isRed)
            Robot.driveTrain.turnOnHeading(turnSpeed, leftTurn, turnTimeOut);
        else
            Robot.driveTrain.turnOnHeading(turnSpeed, rightTurn, turnTimeOut);

        Robot.driveTrain.encoderDrive(driveSpeed, distanceForFirstStone, distanceForFirstStone,driveTimeOut); // This would be 5.5

        Robot.driveTrain.turnOnHeading(turnSpeed, 0, turnTimeOut);

        Robot.driveTrain.encoderDrive(driveSpeed, -6, -6, driveTimeOut);
        //claw down
        //   Robot.claw.timedMove(false, 2);
        Robot.driveTrain.encoderDrive(driveSpeed, 6, 6, driveTimeOut);

        if(isRed)
            Robot.driveTrain.turnOnHeading(turnSpeed, rightTurn, turnTimeOut);
        else
            Robot.driveTrain.turnOnHeading(turnSpeed, leftTurn, turnTimeOut);


        Robot.driveTrain.encoderDrive(1.0, -(50 - distanceForFirstStone), -(50 - distanceForFirstStone), driveTimeOut); // This would be 41.5 - 5.5
        //Horizontal distance from starting to cross line is 13.5 + 28 = 41.5
        //claw up
        if(isRed)
            Robot.driveTrain.turnOnHeading(turnSpeed,rightTurn, turnTimeOut);
        else
            Robot.driveTrain.turnOnHeading(turnSpeed,leftTurn,turnTimeOut);
        //For third case, stop because we can't reach the end of the field.
        if(distanceForFirstStone == 0)
            return;

        Robot.driveTrain.encoderDrive(1.0,  distanceForSecondStone, distanceForSecondStone, driveTimeOut); // This would be 52 + 8


        Robot.driveTrain.turnOnHeading(turnSpeed, 0, turnTimeOut);
        //claw down
        //   Robot.claw.timedMove(false, 2);
        if(isRed)
            Robot.driveTrain.turnOnHeading(turnSpeed, rightTurn, turnTimeOut);
        else
            Robot.driveTrain.turnOnHeading(turnSpeed, leftTurn, turnTimeOut);

        Robot.driveTrain.encoderDrive(1.0, -distanceForSecondStone, -distanceForSecondStone, driveTimeOut);
// Claw move
        Robot.driveTrain.encoderDrive(driveSpeed,7,7,driveTimeOut);
    }

    public void pause(long milliseconds)
    {
        long sleepStart = System.currentTimeMillis();
        while( opModeIsActive() && milliseconds > System.currentTimeMillis() - sleepStart)
        { }
    }
}
