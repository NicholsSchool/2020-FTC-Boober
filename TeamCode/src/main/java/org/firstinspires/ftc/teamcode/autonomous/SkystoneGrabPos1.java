package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;

@Autonomous(name="Skystone From Position 1", group="Loading Zone Autos")
public class SkystoneGrabPos1 extends LinearOpMode {
    private double driveSpeed = 0.5, turnSpeed = 0.4,
            leftTurn = 90, rightTurn = -leftTurn;
    private double driveTimeOut = 3, turnTimeOut = 2;
    private double distanceFromStone = 4.5, distanceAwayFromStone = 5;

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
        run(Robot.colorPicker.isRed(), skyStonePosition);
    }

    private void run(boolean isRed, int skyStonePos)
    {
        Robot.driveTrain.encoderDrive(driveSpeed,   -20.5, -20.5, 3);

        switch(skyStonePos)
        {
            default:
            case 1:
                getStone(isRed, 12.5, 62.5);
                break;
            case 2:
                getStone(isRed, 5.5, 68.5);
                break;
            case 3:
                getStone(isRed, -3.5, 0);
        }
    }



    private void getStone(boolean isRed, double distanceForFirstStone, double distanceForSecondStone)
    {
        if(isRed)
            Robot.driveTrain.turnOnHeading(turnSpeed, leftTurn, turnTimeOut);
        else
            Robot.driveTrain.turnOnHeading(turnSpeed, rightTurn, turnTimeOut);

        Robot.driveTrain.encoderDrive(driveSpeed, distanceForFirstStone, distanceForFirstStone,driveTimeOut); // This would be 5.5

        Robot.driveTrain.turnOnHeading(turnSpeed, 0, turnTimeOut);

        double extraDistance = Robot.distanceSensor.get() - distanceFromStone;
        RobotMap.telemetry.addData("DISTANCE", extraDistance);
        System.out.println("Distance: " + extraDistance);


        Robot.driveTrain.encoderDrive(driveSpeed, -extraDistance, -extraDistance, driveTimeOut);
        //claw down
        Robot.claw.timedMove(false, 1);

        Robot.driveTrain.encoderDrive(driveSpeed, distanceAwayFromStone, distanceAwayFromStone, driveTimeOut, clawDown);

        if(isRed)
            Robot.driveTrain.turnOnHeading(turnSpeed, rightTurn, turnTimeOut);
        else
            Robot.driveTrain.turnOnHeading(turnSpeed, leftTurn, turnTimeOut);


        Robot.driveTrain.encoderDrive(1.0, -(50 - distanceForFirstStone), -(50 - distanceForFirstStone), driveTimeOut); // This would be 41.5 - 5.5
        //Horizontal distance from starting to cross line is 13.5 + 28 = 41.5
        //claw up
        Robot.claw.timedMove(true, 2);
        if(isRed)
            Robot.driveTrain.turnOnHeading(turnSpeed,rightTurn, turnTimeOut);
        else
            Robot.driveTrain.turnOnHeading(turnSpeed,leftTurn,turnTimeOut);
        //For third case, just park because we can't reach the end of the field.
        if(distanceForSecondStone == 0) {
            Robot.driveTrain.encoderDrive(driveSpeed, 7, 7, driveTimeOut);
            return;
        }

        Robot.driveTrain.encoderDrive(1.0,  distanceForSecondStone, distanceForSecondStone, driveTimeOut); // This would be 52 + 8

        Robot.driveTrain.turnOnHeading(turnSpeed, 0, turnTimeOut);

        double extraDistance2 = Robot.distanceSensor.get() - distanceFromStone;
        Robot.driveTrain.encoderDrive(driveSpeed, -extraDistance2, -extraDistance2, driveTimeOut);
        //claw down
        Robot.claw.timedMove(false, 1);

        Robot.driveTrain.encoderDrive(driveSpeed, distanceAwayFromStone, distanceAwayFromStone, driveTimeOut, clawDown);
        if(isRed)
            Robot.driveTrain.turnOnHeading(turnSpeed, rightTurn, turnTimeOut);
        else
            Robot.driveTrain.turnOnHeading(turnSpeed, leftTurn, turnTimeOut);

        Robot.driveTrain.encoderDrive(1.0, -distanceForSecondStone, -distanceForSecondStone, driveTimeOut);
// Claw move
        Robot.claw.timedMove(true, 2);
        Robot.driveTrain.encoderDrive(driveSpeed,7,7,driveTimeOut);
    }

    public void pause(long milliseconds)
    {
        long sleepStart = System.currentTimeMillis();
        while( opModeIsActive() && milliseconds > System.currentTimeMillis() - sleepStart)
        { }
    }
}
