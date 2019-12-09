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
        Robot.driveTrain.halfEncoderDrive(driveSpeed, 12, 12);
        if(isRed)
            Robot.driveTrain.turn(turnSpeed, rightTurn);
        else
            Robot.driveTrain.turn(turnSpeed, leftTurn);
        Robot.driveTrain.halfEncoderDrive(driveSpeed/1.5, 16, 16);
        if(isRed)
            Robot.driveTrain.turn(turnSpeed, rightTurn);
        else
            Robot.driveTrain.turn(turnSpeed, leftTurn);

        Robot.driveTrain.halfEncoderDrive(-driveSpeed, -12.5, -12.5);

        Robot.claw.timedMove(false, 2);
        Robot.driveTrain.halfEncoderDrive(driveSpeed/2,3, 3, true, -0.2);
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
