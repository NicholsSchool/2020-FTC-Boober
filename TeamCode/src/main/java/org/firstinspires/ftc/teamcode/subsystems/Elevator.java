package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;
import org.firstinspires.ftc.teamcode.util.record.Recordable;

public class Elevator extends Subsystem implements Recordable {

    /**
     * Constructs a new Elevator subsystem object
     * @param name - the name for the Subsystem
     */
    public Elevator(String name)
    {
        super(name);
    }

    private void move(double speed)
    {
        RobotMap.elevator.setPower(speed);
    }

    /**
     * Moves the elevator up
     */
    public void up()
    {
        move(1.0);
    }

    /**
     * Moves the elevator down
     */
    public void down()
    {
        move(-1.0);
    }

    @Override
    /**
     * The elevators's gamepad control code for teleop
     */
    public void run()
    {
        if(RobotMap.g2.y)
            up();
        else if(RobotMap.g2.a)
            down();
        else
            stop();
    }

    public void timedMove(boolean goUp, double time)
    {
        RobotMap.timer.reset();
        while(Robot.isAutoRunning() && RobotMap.timer.time() < time)
        {
            if(goUp)
                up();
            else
                down();
        }
        stop();
    }

    @Override
    /**
     * Stops all elevator movement
     */
    public void stop()
    {
        move(0);
    }

    @Override
    /**
     * Returns elevator values to record
     */
    public double[] getValues() {
        return new double[]{RobotMap.elevator.getPower()};
    }

    @Override
    /**
     * Sets elevator servo to given values
     */
    public void setValues(double[] vals) {
       move(vals[0]);
    }
}
