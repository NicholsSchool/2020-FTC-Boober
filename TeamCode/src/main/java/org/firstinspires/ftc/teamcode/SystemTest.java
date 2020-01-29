package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;

@TeleOp(name="System Test", group="Tests")
public class SystemTest extends OpMode {
    @Override
    public void init() {
        Robot.init(hardwareMap, telemetry, gamepad1, gamepad2);
        Robot.initCamera();
        Robot.driveTrain.setBrakeMode(true);
    }
    int skystonePos = 0;
    @Override
    public void loop() {
        double power = 0.5;
        if(RobotMap.g1.y)
        {
            RobotMap.lfDrive.setPower(power);
            RobotMap.lmDrive.setPower(0);
            RobotMap.lbDrive.setPower(0);

            RobotMap.rfDrive.setPower(power);
            RobotMap.rmDrive.setPower(0);
            RobotMap.rbDrive.setPower(0);
            RobotMap.telemetry.addLine("FRONT MOTORS ONLY SPINNING");
        }
        else if(RobotMap.g1.x)
        {
            RobotMap.lfDrive.setPower(0);
            RobotMap.lmDrive.setPower(power);
            RobotMap.lbDrive.setPower(0);

            RobotMap.rfDrive.setPower(0);
            RobotMap.rmDrive.setPower(power);
            RobotMap.rbDrive.setPower(0);
            RobotMap.telemetry.addLine("MIDDLE MOTORS ONLY SPINNING");
        }
        else if(RobotMap.g1.a)
        {
            RobotMap.lfDrive.setPower(0);
            RobotMap.lmDrive.setPower(0);
            RobotMap.lbDrive.setPower(power);

            RobotMap.rfDrive.setPower(0);
            RobotMap.rmDrive.setPower(0);
            RobotMap.rbDrive.setPower(power);
            RobotMap.telemetry.addLine("BACK MOTORS ONLY SPINNING");
        }
        else if(RobotMap.g1.b)
        {
            RobotMap.lfDrive.setPower(power);
            RobotMap.lmDrive.setPower(power);
            RobotMap.lbDrive.setPower(power);

            RobotMap.rfDrive.setPower(power);
            RobotMap.rmDrive.setPower(power);
            RobotMap.rbDrive.setPower(power);
            RobotMap.telemetry.addLine("ALL MOTORS SPINNING");
        }
        else
        {
            RobotMap.lfDrive.setPower(0);
            RobotMap.lmDrive.setPower(0);
            RobotMap.lbDrive.setPower(0);

            RobotMap.rfDrive.setPower(0);
            RobotMap.rmDrive.setPower(0);
            RobotMap.rbDrive.setPower(0);
            RobotMap.telemetry.addLine("ALL MOTORS STOPPED");
        }

      //   RobotMap.lmDrive.setPower(0.5);

        if(RobotMap.g1.dpad_up)
            Robot.driveTrain.resetEncoders();
        if(RobotMap.g1.dpad_down)
            Robot.gyro.resetAngle();
        if(RobotMap.g1.dpad_left)
            Robot.camera.takePhoto(Robot.colorPicker.isRed(), Constants.ROBOT_START_POSITION );
        if(RobotMap.g1.dpad_right) {
            skystonePos = Robot.camera.getSkystonePosition(Robot.colorPicker.isRed(), Constants.ROBOT_START_POSITION);
            System.out.println("Skystone Position: " + skystonePos);
        }
        RobotMap.telemetry.addData("Skystone Position", skystonePos);
        RobotMap.telemetry.addData("Color Sensor, isRed", Robot.colorPicker.isRed());
        Robot.driveTrain.printEncoders();
        Robot.gyro.print();
        Robot.gyro.testPrint();
        Robot.backDistanceSensor.print();
        Robot.frontDistanceSensor.print();
        RobotMap.telemetry.update();
    }
    public void stop() {
        Robot.stop();
    }
}
