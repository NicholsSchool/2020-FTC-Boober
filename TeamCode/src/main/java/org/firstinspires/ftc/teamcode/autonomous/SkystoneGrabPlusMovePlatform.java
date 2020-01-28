package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous(name="Skystone + Platform", group="Loading Zone Autos")
public class SkystoneGrabPlusMovePlatform extends LinearOpMode {
    private double driveSpeed = 1, turnSpeed = 0.4,
            leftTurn = 90, rightTurn = -leftTurn;
    private double driveTimeOut = 3, turnTimeOut = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap,telemetry, gamepad1, gamepad2, this);
        SkystoneGrabPos1Movement stoneMovement = new SkystoneGrabPos1Movement();
        MovePlatformMovement platformMovement = new MovePlatformMovement();
        stoneMovement.setPark(false);
        stoneMovement.runMovement();
        Robot.claw.timedMove(false, 1);
        Robot.driveTrain.encoderDrive(driveSpeed, -26, -26, driveTimeOut);
        Robot.claw.timedMove(true, 1);
        platformMovement.grabPlatform(Robot.colorPicker.isRed());
        Robot.driveTrain.encoderDrive(0.5,45, 45, 3);
    }


}
