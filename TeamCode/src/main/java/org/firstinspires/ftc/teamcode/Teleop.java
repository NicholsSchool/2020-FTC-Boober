package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;

@TeleOp(name="BooberTeleop", group="Iterative Opmode")
public class Teleop extends OpMode
{
    @Override
    public void init() {
        Robot.init(hardwareMap, telemetry, gamepad1, gamepad2);
    }

    @Override
    public void loop() {

        Robot.driveTrain.move(RobotMap.g1.left_stick_y, RobotMap.g1.right_stick_y);
        RobotMap.lGripper.setPower(0.5);

    }

    @Override
    public void stop() {
        Robot.driveTrain.stop();
    }
}
