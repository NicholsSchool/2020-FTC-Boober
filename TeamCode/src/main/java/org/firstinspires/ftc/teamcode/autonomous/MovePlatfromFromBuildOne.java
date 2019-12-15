package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous(name="Platform Move FROM BUILD SIDE OLD", group="Build Side Autos")
public class MovePlatfromFromBuildOne extends LinearOpMode {
    private double driveSpeed = 0.5, turnSpeed = 0.4,
            leftTurn = 80, rightTurn = -leftTurn;
    @Override
    public void runOpMode() {
        Robot.init(hardwareMap, telemetry, gamepad1, gamepad2, this);
        Robot.driveTrain.setBrakeMode(false);
        waitForStart();
        run(Robot.colorPicker.isRed());
    }

    public void run(boolean isRed) {
        Robot.driveTrain.encoderDrive(driveSpeed, 12, 12, 3);
        if(isRed)
            Robot.driveTrain.turn(turnSpeed, rightTurn, 3);
        else
            Robot.driveTrain.turn(turnSpeed, leftTurn, 3);
        Robot.driveTrain.encoderDrive(driveSpeed/1.5, 16, 16, 3);
        if(isRed)
            Robot.driveTrain.turn(turnSpeed, rightTurn, 3);
        else
            Robot.driveTrain.turn(turnSpeed, leftTurn, 3);

        Robot.driveTrain.encoderDrive(-driveSpeed, -12.5, -12.5, 3);

        Robot.claw.timedMove(false, 2);
        Robot.driveTrain.encoderDrive(driveSpeed/2,3, 3, 3);
        Robot.driveTrain.timedMove(driveSpeed, 4);
        Robot.claw.timedMove(true, 2);

//        if(isRed)
//            Robot.driveTrain.turn(turnSpeed, rightTurn + 15);
//        else
//            Robot.driveTrain.turn(turnSpeed*1.5, leftTurn - 10);
//        if(isRed)
//            Robot.driveTrain.halfEncoderDrive(driveSpeed,48, 48);
//        else
//        {
//            Robot.driveTrain.halfEncoderDrive(driveSpeed,48, 48);
//        }

    }
}
