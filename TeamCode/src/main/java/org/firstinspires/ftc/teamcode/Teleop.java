package org.firstinspires.ftc.teamcode;

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

        if(RobotMap.g1.right_bumper)
            Robot.intake.intake();
        else if(RobotMap.g1.left_bumper)
            Robot.intake.outtake();
        else
            Robot.intake.stopTurn();

        if(RobotMap.g1.a)
            Robot.intake.up();
        else if(RobotMap.g1.y)
            Robot.intake.down();
        else
            Robot.intake.stopLift();

    }

    @Override
    public void stop() {
        Robot.driveTrain.stop();
    }
}
