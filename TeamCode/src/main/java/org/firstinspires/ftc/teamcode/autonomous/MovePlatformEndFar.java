package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.movements.MovePlatformMovement;
import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous(name="Waffle Move END FAR", group="Build Side Autos")
public class MovePlatformEndFar extends LinearOpMode {
    private double driveSpeed = 0.75, turnSpeed = 0.4,
            leftTurn = 90, rightTurn = -leftTurn;
    private double driveTimeOut = 3, turnTimeOut = 2;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap,telemetry, gamepad1, gamepad2, this);
        Robot.driveTrain.setBrakeMode(false);
        boolean isRed = Robot.colorPicker.isRed();
        waitForStart();
        new MovePlatformMovement().run(isRed);

        Robot.driveTrain.encoderDrive(driveSpeed,22, 22, driveTimeOut);
        Robot.driveTrain.turnOnHeading(turnSpeed, 0, turnTimeOut);
        Robot.driveTrain.encoderDrive(driveSpeed,-18, -18, driveTimeOut);
        if(isRed)
            Robot.driveTrain.turnOnHeading(turnSpeed, rightTurn, turnTimeOut);
        else
            Robot.driveTrain.turnOnHeading(turnSpeed, leftTurn, turnTimeOut);
        Robot.driveTrain.encoderDrive(driveSpeed,9, 9, driveTimeOut);
    }

}
