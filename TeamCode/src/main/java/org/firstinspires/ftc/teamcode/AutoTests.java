package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;

@Autonomous(name="AutoTests", group="Robot Auto")
public class AutoTests extends LinearOpMode {
    double driveSpeed = 0.5;
    double turnSpeed = 0.4;
    @Override
    public void runOpMode() {
        Robot.init(hardwareMap, FtcDashboard.getInstance().getTelemetry(), gamepad1, gamepad2, this);

        waitForStart();
        RobotMap.telemetry.addLine("About to run Auto Tests");
        RobotMap.telemetry.update();
    //    Robot.driveTrain.turn(turnSpeed, 90, 3); //left
        Robot.driveTrain.turnOnHeading(turnSpeed, 90, 3);
        pause(5000);
        Robot.driveTrain.turnOnHeading(turnSpeed, 0, 3);
//        Robot.driveTrain.encoderDrive(0.5, 36, 36, 10);
//        Robot.driveTrain.turn(turnSpeed, -90, 3); //right
//        Robot.driveTrain.encoderDrive(0.5, 12, 12, 10);
      //  pause(10000);
    }
    public void pause(long milliseconds)
    {
        long sleepStart = System.currentTimeMillis();
        while( opModeIsActive() && milliseconds > System.currentTimeMillis() - sleepStart)
        { }
    }


}
