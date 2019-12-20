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

    /**
     * The elevators's gamepad control code for teleop
     */
    @Override
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
        if(Robot.opMode.opModeIsActive()) {
            RobotMap.timer.reset();
            while (Robot.opMode.opModeIsActive() && RobotMap.timer.time() < time) {
                if (goUp)
                    up();
                else
                    down();
            }
            stop();
        }
    }

    /**
     * Stops all elevator movement
     */
    @Override
    public void stop()
    {
        move(0);
    }

    /**
     * Returns elevator values to record
     */
    @Override
    public double[] getValues() {
        return new double[]{RobotMap.elevator.getPower()};
    }

    /**
     * Sets elevator servo to given values
     */
    @Override
    public void setValues(double[] vals) {
       move(vals[0]);
    }
}
