package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.DRElevator;
import org.firstinspires.ftc.teamcode.autonomous.DRIntake;
import org.firstinspires.ftc.teamcode.autonomous.EncoderDrive;
import org.firstinspires.ftc.teamcode.autonomous.TestGroup;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;

import org.firstinspires.ftc.teamcode.util.commandstructure.Command;
import org.firstinspires.ftc.teamcode.util.commandstructure.CommandGroup;


@Autonomous(name="AutoTests", group="Robot Auto")
public class AutoTests extends LinearOpMode {
    double driveSpeed = 0.5;
    double turnSpeed = 0.4;
    @Override
    public void runOpMode() {
        Robot.init(hardwareMap, FtcDashboard.getInstance().getTelemetry(), gamepad1, gamepad2, this);
      //  Command autoCommand = new EncoderDrive(12, 12, -0.3);
        Command autoCommand = new DRElevator(2, true);
        CommandGroup test = new TestGroup();
        waitForStart();
        test.testPrint();
      //  Robot.driveTrain.encoderDrive(0.8, 12, 12);
    //    RobotMap.telemetry.update();
        while (opModeIsActive()) {
            test.run();
            RobotMap.telemetry.update();
        }
    }
    public void pause(long milliseconds)
    {
        long sleepStart = System.currentTimeMillis();
        while( milliseconds > System.currentTimeMillis() - sleepStart)
        { }
    }


}
