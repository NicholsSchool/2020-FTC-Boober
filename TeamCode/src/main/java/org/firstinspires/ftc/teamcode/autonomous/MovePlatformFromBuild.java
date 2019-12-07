package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.movements.FaceBuildZone;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;

@Autonomous(name="Platform Move FROM BUILD SIDE", group="Build Side Autos")
public class MovePlatformFromBuild extends LinearOpMode {
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
        Robot.driveTrain.halfEncoderDrive(driveSpeed/1.5, 10, 10);
        if(isRed)
            Robot.driveTrain.turn(turnSpeed, rightTurn);
        else
            Robot.driveTrain.turn(turnSpeed, leftTurn);

        Robot.driveTrain.halfEncoderDrive(-driveSpeed, -12, -12);

        Robot.claw.timedMove(false, 2);
        Robot.claw.move(-0.2);
        Robot.driveTrain.encoderDrive(driveSpeed/2,3, 3);
        Robot.driveTrain.encoderDrive(driveSpeed,31, 31);
        Robot.claw.timedMove(true, 2);

        if(isRed)
            Robot.driveTrain.turn(turnSpeed, rightTurn + 15);
        else
            Robot.driveTrain.turn(turnSpeed*1.5, leftTurn - 10);
        if(isRed)
            Robot.driveTrain.encoderDrive(driveSpeed,48, 48);
        else
        {
            Robot.driveTrain.encoderDrive(driveSpeed,48, 48);
            //This is done because the way the robot is built it drifts right
            // So I need the right side to go faster just for this last part
//            RobotMap.timer.reset();
//            while(RobotMap.timer.time() < 3)
//            {
//                Robot.driveTrain.move(-driveSpeed, -driveSpeed - 0.1);
//            }
//            Robot.driveTrain.stop();
        }

    }
}
