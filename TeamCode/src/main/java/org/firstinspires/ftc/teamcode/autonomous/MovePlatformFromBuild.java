package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.movements.FaceBuildZone;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;

@Disabled
@Autonomous(name="Platform Move FROM BUILD SIDE", group="Build Side Autos")
public class MovePlatformFromBuild extends LinearOpMode {
    private double driveSpeed = 0.5, turnSpeed = 0.4,
            leftTurn = 80, rightTurn = -leftTurn;
    @Override
    public void runOpMode() {
        Robot.init(hardwareMap, telemetry, gamepad1, gamepad2, this);
        Robot.driveTrain.setBrakeMode(false);
        waitForStart();
        try {
            run(Robot.colorPicker.isRed());
        }
        catch(Exception e)
        {
            System.out.println(e);
            for(StackTraceElement s : e.getStackTrace())
            {
                RobotMap.telemetry.addLine(s.toString());
            }
        }
    }

    public void run(boolean isRed) {
        Robot.driveTrain.halfEncoderDrive(driveSpeed, 12, 12);
        if(isRed)
            Robot.driveTrain.turn(turnSpeed, rightTurn, 3);
        else
            Robot.driveTrain.turn(turnSpeed, leftTurn, 3);
        if(isRed)
            Robot.driveTrain.halfEncoderDrive(driveSpeed/1.5, 10, 10);
        else
            Robot.driveTrain.halfEncoderDrive(driveSpeed/1.5, 16, 16);

        if(isRed)
            Robot.driveTrain.turn(turnSpeed, rightTurn, 3);
        else
            Robot.driveTrain.turn(turnSpeed, leftTurn, 3);

        Robot.driveTrain.halfEncoderDrive(-driveSpeed, -12, -12);

        Robot.claw.timedMove(false, 2);
        double clawHold = -0.1;
        Robot.driveTrain.halfEncoderDrive(driveSpeed/2,3, 3, true, clawHold);



        if(isRed) {
            Robot.driveTrain.halfEncoderDrive(driveSpeed,6, 6, true, clawHold) ;
            Robot.driveTrain.frontTestMove(-0.7, -0.5, rightTurn, true, clawHold);
        }
        else {
            Robot.driveTrain.halfEncoderDrive(driveSpeed,3, 3, true, clawHold);
            Robot.driveTrain.frontTestMove(-0.3, -1, leftTurn, true, clawHold);
        }
        Robot.claw.timedMove(true, 2);

        if(isRed)
            Robot.driveTrain.turn(turnSpeed, -15, 3);
        else
        {
            Robot.driveTrain.turn(turnSpeed, 15, 3);
        }
        Robot.driveTrain.halfEncoderDrive(driveSpeed,25, 25);

    }
}
