package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.commandstructure.Command;

public class DRElevator extends Command {
    private boolean goUp;
    private double time;
    private long startTime;
    public DRElevator(double time, boolean goUp)
    {
        this.time = time * 1000;
        this.goUp = goUp;
        name = "elevator";
    }


    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        if(goUp)
            Robot.elevator.up();
        else
            Robot.elevator.down();
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime > time;
    }

    @Override
    public void end() {
        Robot.elevator.stop();
    }
}
