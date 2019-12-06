package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.util.RobotMap;
import org.firstinspires.ftc.teamcode.util.record.Recordable;

/**
 * Subsystem for Boober's back claws. These claws are used to pull the Foundation.
 */
public class Claw extends Subsystem implements Recordable {

    /**
     * Constructs a new Claw subsystem object
     * @param name - the name for the Subsystem
     */
    public Claw(String name)
    {
        super(name);
    }


    public void move(double speed)
    {
        RobotMap.claw.setPower(speed);
    }

    /**
     * Moves Boober's back claws upwards
     */
    public void up()
    {
        move(1.0);
    }

    /**
     * Moves Boober's back claws downwards
     */
    public void down()
    {
        move(-1.0);
    }

    public void timedMove(boolean goUp, double time)
    {
        RobotMap.timer.reset();
        while(RobotMap.timer.time() < time) {
            if (goUp)
                up();
            else
                down();
        }
        stop();
    }


    @Override
    /**
     * Claw's gamepad control code for teleop
     */
    public void run()
    {
        if(RobotMap.g2.dpad_up )
            up();
        else if(RobotMap.g2.dpad_down)
            down();
        else
            stop();
    }

    @Override
    /**
     * Stops all claw movements
     */
    public void stop()
    {
        move(0);
    }

    @Override
    /**
     * Returns values to record
     */
    public double[] getValues() {
        return new double[]{RobotMap.claw.getPower()};
    }

    @Override
    /**
     * Sets the claw to the given value
     */
    public void setValues(double[] vals) {
        move(vals[0]);
    }
}
