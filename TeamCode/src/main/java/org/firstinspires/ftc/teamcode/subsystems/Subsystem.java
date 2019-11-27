package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.teamcode.util.Robot;

public abstract class Subsystem {

    private String name;

    public Subsystem(String name)
    {
        this.name = name;
        Robot.registerSubsystem(this);
    }

    public String getName() {
        return name;
    }

    public abstract void run();

    public abstract void stop();
}
