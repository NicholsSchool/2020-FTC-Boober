package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.RobotMap;

@TeleOp(name="BooberTeleop", group="Iterative Opmode")
public class Teleop extends OpMode
{
    @Override
    public void init() {
        RobotMap.init(hardwareMap);
    }

    @Override
    public void loop() {
        RobotMap.lfDrive.setPower(gamepad1.left_stick_y);
        RobotMap.lmDrive.setPower(gamepad1.left_stick_y);
        RobotMap.lbDrive.setPower(gamepad1.left_stick_y);

        RobotMap.rfDrive.setPower(gamepad1.right_stick_y);
        RobotMap.rmDrive.setPower(gamepad1.right_stick_y);
        RobotMap.rbDrive.setPower(gamepad1.right_stick_y);

    }

    @Override
    public void stop() {
        RobotMap.lfDrive.setPower(0);
        RobotMap.lmDrive.setPower(0);
        RobotMap.lbDrive.setPower(0);
        RobotMap.rfDrive.setPower(0);
        RobotMap.rmDrive.setPower(0);
        RobotMap.rbDrive.setPower(0);
    }
}
