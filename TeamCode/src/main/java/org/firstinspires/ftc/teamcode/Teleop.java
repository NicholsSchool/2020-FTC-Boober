package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;


@TeleOp(name="Ok Boober Teleop", group="Iterative Opmode")
/**
 * The game play teleop code to run
 */
public class Teleop extends OpMode
{
    @Override
    /**
     * Intializes the objects within the Robot class
     */
    public void init() {
        Robot.init(hardwareMap, telemetry, gamepad1, gamepad2);
        Robot.driveTrain.setBrakeMode(true);
    }

    @Override
    /**
     * Runs the robot movements
     */
    public void loop() {
        Robot.run();
//        double power = 0.5;
//        if(RobotMap.g1.y)
//        {
//            RobotMap.lfDrive.setPower(power);
//            RobotMap.lmDrive.setPower(0);
//            RobotMap.lbDrive.setPower(0);
//
//            RobotMap.rfDrive.setPower(power);
//            RobotMap.rmDrive.setPower(0);
//            RobotMap.rbDrive.setPower(0);
//            RobotMap.telemetry.addLine("FRONT MOTORS ONLY SPINNING");
//        }
//        else if(RobotMap.g1.x)
//        {
//            RobotMap.lfDrive.setPower(0);
//            RobotMap.lmDrive.setPower(power);
//            RobotMap.lbDrive.setPower(0);
//
//            RobotMap.rfDrive.setPower(0);
//            RobotMap.rmDrive.setPower(power);
//            RobotMap.rbDrive.setPower(0);
//            RobotMap.telemetry.addLine("MIDDLE MOTORS ONLY SPINNING");
//        }
//        else if(RobotMap.g1.a)
//        {
//            RobotMap.lfDrive.setPower(0);
//            RobotMap.lmDrive.setPower(0);
//            RobotMap.lbDrive.setPower(power);
//
//            RobotMap.rfDrive.setPower(0);
//            RobotMap.rmDrive.setPower(0);
//            RobotMap.rbDrive.setPower(power);
//            RobotMap.telemetry.addLine("BACK MOTORS ONLY SPINNING");
//        }
//        else if(RobotMap.g1.b)
//        {
//            RobotMap.lfDrive.setPower(power);
//            RobotMap.lmDrive.setPower(power);
//            RobotMap.lbDrive.setPower(power);
//
//            RobotMap.rfDrive.setPower(power);
//            RobotMap.rmDrive.setPower(power);
//            RobotMap.rbDrive.setPower(power);
//            RobotMap.telemetry.addLine("ALL MOTORS SPINNING");
//        }
//        else
//        {
//            RobotMap.lfDrive.setPower(0);
//            RobotMap.lmDrive.setPower(0);
//            RobotMap.lbDrive.setPower(0);
//
//            RobotMap.rfDrive.setPower(0);
//            RobotMap.rmDrive.setPower(0);
//            RobotMap.rbDrive.setPower(0);
//            RobotMap.telemetry.addLine("ALL MOTORS STOPPED");
//        }
        Robot.driveTrain.printEncoders();
        if(RobotMap.g1.a)
            Robot.driveTrain.resetEncoders();
        if(RobotMap.g1.y)
            Robot.gyro.resetAngle();
        if(RobotMap.g1.x)
            Robot.camera.takePhoto(Robot.colorPicker.isRed(), Constants.ROBOT_START_POSITION );
        Robot.gyro.print();
        RobotMap.telemetry.update();
    }

    @Override
    /**
     * Stops all Robot movements
     */
    public void stop() {
        Robot.stop();
    }
}
