package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;
import org.firstinspires.ftc.teamcode.util.record.Recordable;

public class Claw extends Subsystem implements Recordable {

    public Claw(String name)
    {
        super(name);
    }


    private void move(double speed)
    {
        RobotMap.claw.setPower(speed);
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
        if(RobotMap.g1.dpad_up )
            up();
        else if(RobotMap.g1.dpad_down)
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
        return new double[]{RobotMap.claw.getPower()};
    }

    @Override
    public void setValues(double[] vals) {
        move(vals[0]);
    }
}
