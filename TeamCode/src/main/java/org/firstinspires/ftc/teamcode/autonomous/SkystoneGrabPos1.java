package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
        int skyStonePosition = 1;
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
                stoneOne(isRed);
                break;
            case 2:
                stoneTwo(isRed);
                break;
            case 3:
                stoneThree(isRed);
        }
    }

    private void stoneOne(boolean isRed)
    {
        Robot.driveTrain.turn(turnSpeed, rightTurn, 3);
        Robot.driveTrain.encoderDrive(driveSpeed, 13.5, 13.5,3);
        Robot.driveTrain.turn(turnSpeed, leftTurn, 3);
        Robot.driveTrain.encoderDrive(driveSpeed, -6, -6, 3);
        //claw down
     //   Robot.claw.timedMove(false, 2);
        Robot.driveTrain.encoderDrive(driveSpeed, 6, 6, 3);
        Robot.driveTrain.turn(turnSpeed, leftTurn, 3);
        Robot.driveTrain.encoderDrive(driveSpeed, -28, -28, 3);
        //claw up
        Robot.driveTrain.encoderDrive(driveSpeed, 52, 52, 3);
        Robot.driveTrain.turn(turnSpeed, rightTurn, 3);
        //claw down
        //   Robot.claw.timedMove(false, 2);
        Robot.driveTrain.turn(turnSpeed, leftTurn, 3);
        Robot.driveTrain.encoderDrive(driveSpeed, -50, -50, 3);

    }

    private void stoneTwo(boolean isRed)
    {}
    private void stoneThree(boolean isRed)
    {}
}
