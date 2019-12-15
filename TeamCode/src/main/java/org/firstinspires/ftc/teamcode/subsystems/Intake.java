package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;
import org.firstinspires.ftc.teamcode.util.record.Recordable;

public class Intake extends Subsystem implements Recordable {

    /**
     * Constructs a new Intake Subsystem object
     * @param name - the name of the Subsystem
     */
    public Intake(String name)
    {
        super(name);
    }

    private void move(double speed)
    {
        RobotMap.lGripper.setPower(speed);
        RobotMap.rGripper.setPower(speed);
    }

    /**
     * Runs intake movement slowly
     */
    public void slowIntake()
    {
        move(Constants.SLOW_INTAKE_SPEED);
    }

    /**
     * Runs intake movement fast
     */
    public void fastIntake()
    {
        move(Constants.FAST_INTAKE_SPEED);
    }

    /**
     * Runs outtake movement fast
     */
    public void fastOuttake()
    {
        move(Constants.FAST_OUTTAKE_SPEED);
    }

    /**
     * Runs outtake movement slowly
     */
    public void slowOuttake()
    {
        move(Constants.SLOW_OUTTAKE_SPEED);
    }

    public void timedMove(boolean intake, boolean fast, double time)
    {
        if(Robot.opMode.opModeIsActive()) {
            RobotMap.timer.reset();
            while (Robot.opMode.opModeIsActive() && RobotMap.timer.time() < time) {
                if (intake) {
                    if (fast)
                        fastIntake();
                    else
                        slowIntake();
                } else {
                    if (fast)
                        fastOuttake();
                    else
                        slowOuttake();
                }
            }
            stop();
        }
    }

    @Override
    /**
     * The gripper's gamepad control code for teleop
     */
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
    /**
     * Stops all gripper movements
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
        return new double[]{RobotMap.lGripper.getPower(), RobotMap.rGripper.getPower()};
    }

    @Override
    /**
     * Sets gripper motors to the given values.
     */
    public void setValues(double[] vals) {
        RobotMap.lGripper.setPower(vals[0]);
        RobotMap.lGripper.setPower(vals[1]);
    }
}
