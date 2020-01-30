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

    Robot.driveTrain.turnOnHeading(turnSpeed, 90, turnTimeOut);


//        if(isRed)
//            Robot.driveTrain.firmEncoderDrive(driveSpeed, 20, 5, 10, clawDown);
//        else
//            Robot.driveTrain.firmEncoderDrive(driveSpeed, 5, 20, 10, clawDown);
//
//        if(isRed)
//            Robot.driveTrain.driveAndTurn(fastSide, slowSide, rightTurn, turnTimeOut, clawDown);
//        else
//            Robot.driveTrain.driveAndTurn(slowSide, fastSide, leftTurn, turnTimeOut, clawDown);
//
//        Robot.claw.timedMove(true, 1.5);
//        Robot.driveTrain.encoderDrive(driveSpeed, -5, -5, driveTimeOut);
//
//        if(isRed)
//            Robot.driveTrain.turnOnHeading(turnSpeed, rightTurn, turnTimeOut );
//        else
//            Robot.driveTrain.turnOnHeading(turnSpeed, leftTurn, turnTimeOut );
//
//        Robot.driveTrain.encoderDrive(driveSpeed, 15, 15, driveTimeOut);

//        double turnGap = 3;
////        if(skystonePos == 2)
////            turnGap = 3.5;
////
////        double desiredDistanceFromWall = (6 - skystonePos) * skystoneLength - turnGap;
////        double currentDistanceFromWall = Robot.backDistanceSensor.get();
////
////        double distanceToTravel = desiredDistanceFromWall - currentDistanceFromWall;
////
////
////        Robot.driveTrain.encoderDrive(driveSpeed,distanceToTravel,distanceToTravel,3);
////        Robot.driveTrain.turnOnHeading(turnSpeed, 90, turnTimeOut);
////
////        double extraDistance = Robot.backDistanceSensor.get() - distanceFromStone;
////
////        Robot.driveTrain.encoderDrive(driveSpeed, -extraDistance, -extraDistance, driveTimeOut);
////        //claw down
////        Robot.claw.timedMove(false, 1);
////
////        Robot.driveTrain.encoderDrive(driveSpeed, distanceAwayFromStone, distanceAwayFromStone, driveTimeOut);
////        Robot.driveTrain.turnOnHeading(turnSpeed, 0, turnTimeOut, clawDown);
////
////        RobotMap.telemetry.addData("Distance from Wall As Detected", currentDistanceFromWall);
////        RobotMap.telemetry.addData("The Desired Distance From Wall", desiredDistanceFromWall);
////        RobotMap.telemetry.addData("The distance To be traveled", distanceToTravel);
////        RobotMap.telemetry.update();

     //   Robot.driveTrain.encoderDrive(0.5, 24, 24, 3);
//       Robot.driveTrain.turnOnHeading(turnSpeed,90,2);
//        pause(3000);
//        Robot.driveTrain.turnOnHeading(turnSpeed,0,2);
    }
    public void pause(long milliseconds)
    {
        long sleepStart = System.currentTimeMillis();
        while( opModeIsActive() && milliseconds > System.currentTimeMillis() - sleepStart)
        { }
    }


}
