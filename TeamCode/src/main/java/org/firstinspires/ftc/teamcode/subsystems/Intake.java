package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.util.RobotMap;

public class Intake {

    private void turn(double speed)
    {
        RobotMap.lGripper.setPower(speed);
        RobotMap.rGripper.setPower(speed);
    }

    public void intake()
    {
        turn(1.0);
    }

    public void outtake()
    {
        turn(-1.0);
    }

    private void lift(double speed)
    {
        RobotMap.lClaw.setPower(speed);
        RobotMap.rClaw.setPower(speed);
    }

    public void up()
    {
        lift(1.0);
    }

    public void down()
    {
        lift(-1.0);
    }

    public void stopLift()
    {
        lift(0);
    }

    public void stopTurn()
    {
        turn(0);
    }

    public void stop()
    {
        stopLift();
        stopTurn();
    }
}
