package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;
import org.firstinspires.ftc.teamcode.util.record.Recordable;

import java.util.function.Consumer;

public class DriveTrain extends Subsystem implements Recordable {


    private static final double
            COUNTS_PER_REV  = -530, INCHES_PER_REV = 4 * Math.PI, COUNTS_PER_INCH = COUNTS_PER_REV/INCHES_PER_REV;
    DcMotorSimple[] motors;
    public DriveTrain(String name)
    {
        super(name);
        motors = new DcMotorSimple[6];
        motors[0] = RobotMap.lfDrive;
        motors[1] = RobotMap.lmDrive;
        motors[2] = RobotMap.lbDrive;
        motors[3] = RobotMap.rfDrive;
        motors[4] = RobotMap.rmDrive;
        motors[5] = RobotMap.rbDrive;
    }

    public void move(double lSpeed, double rSpeed)
    {
        lSpeed *= Math.abs(lSpeed);
        rSpeed *= Math.abs(rSpeed);
        lSpeed *= Constants.DRIVE_BUFFER;
        rSpeed *= Constants.DRIVE_BUFFER;

       for(int i = 0; i < motors.length; i++)
           if(i < motors.length/2)
               motors[i].setPower(lSpeed);
           else
               motors[i].setPower(rSpeed);


    }

    public void encoderTest()
    {
        for(int i = 0;  i < motors.length; i ++) {
            if(!(motors[i] instanceof DcMotor) )
                continue;
            DcMotor m = (DcMotor) motors[i];
            RobotMap.telemetry.addData((i < motors.length/2 ? "left " : "right ") + "Encoder " + i, m.getCurrentPosition());
        }
    }



    public void encoderDrive(double speed, double leftInches, double rightInches) {
            for(int i = 0; i < motors.length; i ++)
            {
                if(!(motors[i] instanceof  DcMotor))
                    continue;
                DcMotor m = (DcMotor) motors[i];
                int target = m.getCurrentPosition() + (int)((i < motors.length/2 ? leftInches : rightInches) * COUNTS_PER_INCH);
                m.setTargetPosition(target);
                m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                m.setPower(Math.abs(speed));
            }


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            boolean isBusy = true;
            boolean leftIsBusy = true, rightIsBusy = true;
            while (Robot.opMode.opModeIsActive() && (leftIsBusy || rightIsBusy)) {

                for(int i = 0; i < motors.length; i ++)
                {
                    if(!(motors[i] instanceof  DcMotor))
                        continue;
                    DcMotor m = (DcMotor) motors[i];
                    if(i < motors.length)
                        leftIsBusy = leftIsBusy && m.isBusy();
                    else
                        rightIsBusy = rightIsBusy && m.isBusy();
                }
            }

            // Stop all motion;
        // Turn off RUN_TO_POSITION
            for(int i = 0; i < motors.length; i ++)
            {
                if(!(motors[i] instanceof  DcMotor))
                    continue;
                DcMotor m = (DcMotor) motors[i];
                m.setPower(0);
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Check what mode it should be
            }



    }

    public void colorDrive(double speed)
    {
        speed *= -1;
        boolean allGood = false;
        boolean left = true, right = true;
        while(Robot.opMode.opModeIsActive() && !allGood)
        {
            allGood = true;
            for(int i = 0; i < motors.length; i ++)
                if(left && i < motors.length/2)
                {
                    if(RobotMap.lColorSensor.red() < 40) {
                        motors[i].setPower(speed);
                        allGood = false;
                    }
                    else
                        left = false;

                }
                else if(right) {
                    if (right && RobotMap.rColorSensor.red() < 40) {
                        motors[i].setPower(speed);
                        allGood = false;
                    }
                    else
                        right = false;
                }

        }
        stop();

    }

    public void resetEncoders()
    {
        for(int i = 0; i < motors.length; i ++)
        {
            if(!(motors[i] instanceof  DcMotor))
                continue;
            DcMotor m = (DcMotor) motors[i];
            m.setPower(0);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void run()
    {
            move(RobotMap.g1.left_stick_y, RobotMap.g1.right_stick_y);


        RobotMap.telemetry.addData("Run Mode", RobotMap.rmDrive.getMode());
/*        encoderTest();
        if(RobotMap.g1.dpad_left)
            resetEncoders();*/


    }

    @Override
    public void stop()
    {
        move(0,0);
    }

    @Override
    public double[] getValues() {
        return new double[]{};
    }

    @Override
    public void setValues(double[] vals) {

    }
}
