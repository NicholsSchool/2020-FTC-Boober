package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.Function;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;

@Autonomous(name="AutoTests", group="Robot Auto")
public class AutoTests extends LinearOpMode {
    double driveSpeed = 0.5;
    double turnSpeed = 0.4,     leftTurn = 90, rightTurn = -leftTurn;
    double skystoneLength = 8;
    double skystonePos = Constants.TEST_SKYSTONE_POSITION;

    double fastSide = 0.6, slowSide = 0.2;


    double turnTimeOut = 2,  driveTimeOut = 3;
    private double distanceFromStone = 4.5, distanceAwayFromStone = 7;

    Function clawDown = new Function() {
        public void execute(){Robot.claw.down();}
        public void stop(){Robot.claw.stop();}
    };

    @Override
    public void runOpMode() {
        Robot.init(hardwareMap, telemetry, gamepad1, gamepad2, this);

        waitForStart();
        RobotMap.telemetry.addLine("About to run Auto Tests");
        RobotMap.telemetry.update();
//        Robot.claw.timedMove(false, 2);
    boolean isRed = Robot.colorPicker.isRed();

        Robot.claw.timedMove(true, 1);

    }
    public void pause(long milliseconds)
    {
        long sleepStart = System.currentTimeMillis();
        while( opModeIsActive() && milliseconds > System.currentTimeMillis() - sleepStart)
        { }
    }


}
