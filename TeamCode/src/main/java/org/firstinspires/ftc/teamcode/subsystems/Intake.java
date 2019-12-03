package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;
import org.firstinspires.ftc.teamcode.util.record.Recordable;

public class Intake extends Subsystem implements Recordable {

    public Intake(String name)
    {
        super(name);
    }

    private void move(double speed)
    {
        RobotMap.lGripper.setPower(speed);
        RobotMap.rGripper.setPower(speed);
    }

    public void intake(double speed)
    {
        move(Math.abs(speed));
    }

    public void outtake(double speed)
    {
        move(-Math.abs(speed));
    }

    public void slowIntake()
    {
        intake(Constants.SLOW_INTAKE_SPEED);
    }

    public void fastIntake()
    {
        intake(Constants.FAST_INTAKE_SPEED);
    }

    public void fastOuttake()
    {
        outtake(Constants.FAST_OUTTAKE_SPEED);
    }

    public void slowOuttake()
    {
        outtake(Constants.SLOW_OUTTAKE_SPEED);
    }

    @Override
    public void run()
    {
        if(RobotMap.g2.right_bumper)
            slowIntake();
        else if (RobotMap.g2.right_trigger>0.5)
            fastIntake();
        else if(RobotMap.g2.left_bumper)
            slowOuttake();
        else if (RobotMap.g2.left_trigger>0.5)
            fastOuttake();
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
        return new double[]{RobotMap.lGripper.getPower(), RobotMap.rGripper.getPower()};
    }

    @Override
    public void setValues(double[] vals) {
        RobotMap.lGripper.setPower(vals[0]);
        RobotMap.lGripper.setPower(vals[1]);
    }
}
