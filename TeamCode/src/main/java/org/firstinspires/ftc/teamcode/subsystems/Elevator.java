package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.util.RobotMap;
import org.firstinspires.ftc.teamcode.util.record.Recordable;

public class Elevator extends Subsystem implements Recordable {

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
        if(RobotMap.g2.y)
            up();
        else if(RobotMap.g2.a)
            down();
        else
            stop();
    }

    @Override
    public void stop()
    {
        move(0);
    }

    @Override
    public double[] getValues() {
        return new double[]{RobotMap.elevator.getPower()};
    }

    @Override
    public void setValues(double[] vals) {
       move(vals[0]);
    }
}
