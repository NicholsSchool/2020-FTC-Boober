package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;

public class Intake extends Subsystem{

    public Intake(String name)
    {
        super(name);
    }

    private void move(double speed)
    {
        RobotMap.lGripper.setPower(speed);
        RobotMap.rGripper.setPower(speed);
    }

    public void slowIntake()
    {
            move(Constants.SLOW_INTAKE_SPEED);
    }

    public void fastIntake()
    {
        move(Constants.FAST_INTAKE_SPEED);
    }

    public void fastOuttake()
    {
        move(Constants.FAST_OUTTAKE_SPEED);
    }

    public void slowOuttake()
    {
        move(Constants.SLOW_OUTTAKE_SPEED);
    }

    @Override
    public void run()
    {
        if(RobotMap.g1.right_bumper)
            slowIntake();
        else if (RobotMap.g1.right_trigger>0.5)
            fastIntake();
        else if(RobotMap.g1.left_bumper)
            fastOuttake();
        else if (RobotMap.g1.left_trigger>0.5)
            slowOuttake();
        else
            stop();
    }

    @Override
    public void stop()
    {
       move(0);
    }
}
