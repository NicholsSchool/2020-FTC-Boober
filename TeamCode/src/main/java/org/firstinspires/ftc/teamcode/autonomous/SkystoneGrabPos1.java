package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous(name="Skystone From Position 1", group="Loading Zone Autos")
public class SkystoneGrabPos1 extends LinearOpMode {
    private double driveSpeed = 0.5, turnSpeed = 0.4,
            leftTurn = 90, rightTurn = -leftTurn;

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
                newGetStone(isRed, 13.5, 52);
                break;
            case 2:
                newGetStone(isRed, 5.5, 60);
                break;
            case 3:
                newGetStone(isRed, -3.5, 0);
        }
    }

    private void getStone(boolean isRed, double distanceForFirstStone, double distanceForSecondStone)
    {
        if(isRed)
            Robot.driveTrain.turn(turnSpeed, leftTurn, 3);
        else
            Robot.driveTrain.turn(turnSpeed, rightTurn, 3);

        Robot.driveTrain.encoderDrive(driveSpeed, distanceForFirstStone, distanceForFirstStone,3); // This would be 5.5

        if(isRed)
            Robot.driveTrain.turn(turnSpeed, rightTurn, 3);
        else
            Robot.driveTrain.turn(turnSpeed, leftTurn, 3);

        Robot.driveTrain.encoderDrive(driveSpeed, -6, -6, 3);
        //claw down
        //   Robot.claw.timedMove(false, 2);
        Robot.driveTrain.encoderDrive(driveSpeed, 6, 6, 3);

        if(isRed)
            Robot.driveTrain.turn(turnSpeed, rightTurn, 3);
        else
            Robot.driveTrain.turn(turnSpeed, leftTurn, 3);


        Robot.driveTrain.encoderDrive(driveSpeed, -(41.5 - distanceForFirstStone), -(41.5 - distanceForFirstStone), 3); // This would be 41.5 - 5.5
        //Horizontal distance from starting to cross line is 13.5 + 28 = 41.5
        //claw up

        //For third case, stop because we can't reach the end of the field.
        if(distanceForFirstStone == 0)
            return;

        Robot.driveTrain.encoderDrive(driveSpeed,  distanceForSecondStone, distanceForSecondStone, 3); // This would be 52 + 8

        if(isRed)
            Robot.driveTrain.turn(turnSpeed, leftTurn, 3);
        else
            Robot.driveTrain.turn(turnSpeed, rightTurn, 3);
        //claw down
        //   Robot.claw.timedMove(false, 2);
        if(isRed)
            Robot.driveTrain.turn(turnSpeed, rightTurn, 3);
        else
            Robot.driveTrain.turn(turnSpeed, leftTurn, 3);
        Robot.driveTrain.encoderDrive(driveSpeed, -distanceForSecondStone, -distanceForSecondStone, 3);

    }

    private void newGetStone(boolean isRed, double distanceForFirstStone, double distanceForSecondStone)
    {
        if(isRed)
            Robot.driveTrain.turnOnHeading(turnSpeed, leftTurn, 3);
        else
            Robot.driveTrain.turnOnHeading(turnSpeed, rightTurn, 3);

        Robot.driveTrain.encoderDrive(driveSpeed, distanceForFirstStone, distanceForFirstStone,3); // This would be 5.5

        Robot.driveTrain.turnOnHeading(turnSpeed, 0, 3);

        Robot.driveTrain.encoderDrive(driveSpeed, -6, -6, 3);
        //claw down
        //   Robot.claw.timedMove(false, 2);
        Robot.driveTrain.encoderDrive(driveSpeed, 6, 6, 3);

        if(isRed)
            Robot.driveTrain.turnOnHeading(turnSpeed, rightTurn, 3);
        else
            Robot.driveTrain.turnOnHeading(turnSpeed, leftTurn, 3);


        Robot.driveTrain.encoderDrive(driveSpeed, -(41.5 - distanceForFirstStone), -(41.5 - distanceForFirstStone), 3); // This would be 41.5 - 5.5
        //Horizontal distance from starting to cross line is 13.5 + 28 = 41.5
        //claw up

        //For third case, stop because we can't reach the end of the field.
        if(distanceForFirstStone == 0)
            return;

        Robot.driveTrain.encoderDrive(driveSpeed,  distanceForSecondStone, distanceForSecondStone, 3); // This would be 52 + 8


        Robot.driveTrain.turnOnHeading(turnSpeed, 0, 3);
        //claw down
        //   Robot.claw.timedMove(false, 2);
        if(isRed)
            Robot.driveTrain.turnOnHeading(turnSpeed, rightTurn, 3);
        else
            Robot.driveTrain.turnOnHeading(turnSpeed, leftTurn, 3);

        Robot.driveTrain.encoderDrive(driveSpeed, -distanceForSecondStone, -distanceForSecondStone, 3);

    }
    private void stoneOne(boolean isRed)
    {
        if(isRed)
            Robot.driveTrain.turn(turnSpeed, leftTurn, 3);
        else
            Robot.driveTrain.turn(turnSpeed, rightTurn, 3);
        Robot.driveTrain.encoderDrive(driveSpeed, 13.5, 13.5,3); // This would be 5.5

        if(isRed)
            Robot.driveTrain.turn(turnSpeed, rightTurn, 3);
        else
            Robot.driveTrain.turn(turnSpeed, leftTurn, 3);

        Robot.driveTrain.encoderDrive(driveSpeed, -6, -6, 3);
        //claw down
     //   Robot.claw.timedMove(false, 2);
        Robot.driveTrain.encoderDrive(driveSpeed, 6, 6, 3);

        if(isRed)
            Robot.driveTrain.turn(turnSpeed, rightTurn, 3);
        else
            Robot.driveTrain.turn(turnSpeed, leftTurn, 3);


        Robot.driveTrain.encoderDrive(driveSpeed, -28, -28, 3); // This would be 41.5 - 5.5
        //Horizontal distance from starting to cross line is 13.5 + 28 = 41.5
        //claw up
        Robot.driveTrain.encoderDrive(driveSpeed, 52, 52, 3); // This would be 52 + 8

        if(isRed)
            Robot.driveTrain.turn(turnSpeed, leftTurn, 3);
        else
            Robot.driveTrain.turn(turnSpeed, rightTurn, 3);
        //claw down
        //   Robot.claw.timedMove(false, 2);
        if(isRed)
            Robot.driveTrain.turn(turnSpeed, rightTurn, 3);
        else
            Robot.driveTrain.turn(turnSpeed, leftTurn, 3);
        Robot.driveTrain.encoderDrive(driveSpeed, -50, -50, 3);

    }

    private void stoneTwo(boolean isRed)
    {}
    private void stoneThree(boolean isRed)
    {}
    public void pause(long milliseconds)
    {
        long sleepStart = System.currentTimeMillis();
        while( opModeIsActive() && milliseconds > System.currentTimeMillis() - sleepStart)
        { }
    }
}
