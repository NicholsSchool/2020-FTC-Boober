package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.commandstructure.Command;

public class DRIntake extends Command {

    private double speed, time;
    private long startTime;
    public DRIntake(double speed, double time)
    {
        this.speed = speed;
        this.time = time * 1000;
        name = "intake";
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        Robot.intake.intake(speed);
    }

    @Override
    public boolean isFinished() {
        System.out.println(System.currentTimeMillis() - startTime);
        return System.currentTimeMillis() - startTime > time;
    }

    @Override
    public void end() {
        Robot.intake.stop();
    }
}
