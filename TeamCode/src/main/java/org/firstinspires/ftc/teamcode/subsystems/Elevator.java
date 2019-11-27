package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.util.RobotMap;

public class Elevator extends  Subsystem{

    public Elevator(String name)
    {
        super(name);
    }

    private void move(double speed)
    {
        RobotMap.elevator.setPower(speed);
    }

    public void up()
    {
        move(1.0);
    }

    public void down()
    {
        move(-1.0);
    }

    @Override
    public void run()
    {
        if(RobotMap.g1.y)
            up();
        else if(RobotMap.g1.a)
            down();
        else
            stop();
    }

    @Override
    public void stop()
    {
        move(0);
    }

}
