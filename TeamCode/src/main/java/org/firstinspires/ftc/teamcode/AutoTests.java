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

  //      Robot.driveTrain.encoderDrive(0.8,20,20,3);
//
       Robot.driveTrain.turnOnHeading(turnSpeed,90,2);
        pause(3000);
        Robot.driveTrain.turnOnHeading(turnSpeed,0,2);
    }
    public void pause(long milliseconds)
    {
        long sleepStart = System.currentTimeMillis();
        while( opModeIsActive() && milliseconds > System.currentTimeMillis() - sleepStart)
        { }
    }


}
