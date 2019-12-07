package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;

@Autonomous(name="AutoTests", group="Robot Auto")
public class AutoTests extends LinearOpMode {
    double driveSpeed = 0.5;
    double turnSpeed = 0.4;
    @Override
    public void runOpMode() {
        Robot.init(hardwareMap, telemetry, gamepad1, gamepad2, this);

        waitForStart();
        Robot.claw.timedMove(false, 2);
        Robot.claw.move(-0.2);
        Robot.driveTrain.halfEncoderDrive(driveSpeed/2,3, 3);
        Robot.driveTrain.halfEncoderDrive(driveSpeed,12, 12);
        Robot.driveTrain.testMove(-0.7, -0.5, -80);
        Robot.claw.timedMove(true, 2);
        Robot.driveTrain.turn(turnSpeed, -10);
        Robot.driveTrain.halfEncoderDrive(driveSpeed,30, 30);
    }
    public void pause(long milliseconds)
    {
        long sleepStart = System.currentTimeMillis();
        while( milliseconds > System.currentTimeMillis() - sleepStart)
        { }
    }


}
