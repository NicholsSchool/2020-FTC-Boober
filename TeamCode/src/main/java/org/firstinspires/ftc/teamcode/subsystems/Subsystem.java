package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.teamcode.util.Robot;

public abstract class Subsystem {

    private String name;

    /**
     * Constructs a new Subsystem object
     * @param name - the name for the Subsystem
     */
    public Subsystem(String name)
    {
        this.name = name;
        Robot.registerSubsystem(this);
    }

    /**
     * Returns the name of the Subsystem
     * @return the name of the Subsystem
     */
    public String getName() {
        return name;
    }

    /**
     * The gamepad control of the Subsystem for Teleop
     */
    public abstract void run();

    /**
     * Stops all movement for the Subsystem
     */
    public abstract void stop();
}
