package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;
import org.firstinspires.ftc.teamcode.util.commandstructure.Command;

public class EncoderDrive extends Command {
    private double lInches, rInches, speed;

    public EncoderDrive(double leftInches, double rightInches, double speed)
    {
        this.lInches = leftInches;
        this.rInches = rightInches;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        Robot.driveTrain.setEncoderTargets(lInches, rInches, speed);
    }

    @Override
    public void execute(){
    }

    @Override
    public boolean isFinished() {
        RobotMap.telemetry.addLine("Not Done");
        boolean[] busyMotors = Robot.driveTrain.areMotorsBusy();
        return busyMotors[0] || busyMotors[1];
    }

    @Override
    public void end() {
        Robot.driveTrain.turnOffEncoders();
    }
}
