package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;
import org.firstinspires.ftc.teamcode.util.record.Recordable;

import java.util.function.Consumer;

public class DriveTrain extends Subsystem implements Recordable {


    private static final double
            COUNTS_PER_REV  = -530, INCHES_PER_REV = 4 * Math.PI, COUNTS_PER_INCH = COUNTS_PER_REV/INCHES_PER_REV;
    private MotorEntry[] motors;
    private boolean isArcadeDrive;
    public DriveTrain(String name)
    {
        super(name);
        motors = new MotorEntry[6];
        motors[0] = new MotorEntry(RobotMap.lfDrive, true);
        motors[1] = new MotorEntry(RobotMap.lmDrive, true);
        motors[2] = new MotorEntry(RobotMap.lbDrive, true);
        motors[3] = new MotorEntry(RobotMap.rfDrive, false);
        motors[4] = new MotorEntry(RobotMap.rmDrive, false);
        motors[5] = new MotorEntry(RobotMap.rbDrive, false);
        isArcadeDrive = false;
    }

    public class MotorEntry {
        public DcMotorSimple motor;
        public boolean isLeft;
        public MotorEntry(DcMotorSimple motor, boolean isLeft)
        {
            this.motor = motor;
            this.isLeft = isLeft;
        }
    }

    public void move(double lSpeed, double rSpeed)
    {
        lSpeed = Range.clip(lSpeed, -1, 1);
        rSpeed = Range.clip(rSpeed, -1, 1);

        // The x^2 movement Dillan wanted.
        lSpeed *= Math.abs(lSpeed);
        rSpeed *= Math.abs(rSpeed);

        lSpeed *= Constants.DRIVE_BUFFER;
        rSpeed *= Constants.DRIVE_BUFFER;

        for(MotorEntry entry : motors)
            if(entry.isLeft)
                entry.motor.setPower(lSpeed);
            else
                entry.motor.setPower(rSpeed);

    }

    public void tankDrive()
    {
        move(RobotMap.g1.left_stick_y, RobotMap.g1.right_stick_y);
    }

    public void arcadeDrive()
    {
        move( RobotMap.g1.right_stick_y - RobotMap.g1.left_stick_x, RobotMap.g1.right_stick_y + RobotMap.g1.left_stick_x);
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

    public void setEncoderTargets(double speed, double leftInches, double rightInches)
    {
        for(MotorEntry entry : motors) {
            if (!(entry.motor instanceof DcMotor))
                continue;
            DcMotor m = (DcMotor) entry.motor;
            m.setTargetPosition(m.getCurrentPosition() + (int)((entry.isLeft ? leftInches : rightInches) * COUNTS_PER_INCH));
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(Math.abs(speed));
        }
    }

    public boolean[] areMotorsBusy()
    {
        boolean leftIsBusy = true, rightIsBusy = true;
        for(MotorEntry entry : motors)
        {
            if(!(entry.motor instanceof  DcMotor))
                continue;
            DcMotor m = (DcMotor) entry.motor;
            if(entry.isLeft)
                leftIsBusy = leftIsBusy && m.isBusy();
            else
                rightIsBusy = rightIsBusy && m.isBusy();
        }
        return new boolean[]{leftIsBusy, rightIsBusy};
    }

    public void turnOffEncoders()
    {
        // Stop all motion;
        // Turn off RUN_TO_POSITION
        for(MotorEntry entry : motors)
        {
            if(!(entry.motor instanceof  DcMotor))
                continue;
            DcMotor m = (DcMotor) entry.motor;
            m.setPower(0);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Check what mode it should be
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
//        speed *= -1;
//        boolean allGood = false;
//        boolean left = true, right = true;
//        while(Robot.opMode.opModeIsActive() && !allGood)
//        {
//            allGood = true;
//            for(int i = 0; i < motors.length; i ++)
//                if(left && i < motors.length/2)
//                {
//                    if(RobotMap.lColorSensor.red() < 40) {
//                        motors[i].setPower(speed);
//                        allGood = false;
//                    }
//                    else
//                        left = false;
//
//                }
//                else if(right) {
//                    if (right && RobotMap.rColorSensor.red() < 40) {
//                        motors[i].setPower(speed);
//                        allGood = false;
//                    }
//                    else
//                        right = false;
//                }
//
//        }
//        stop();

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

        if(isArcadeDrive)
            arcadeDrive();
        else
            tankDrive();

        if(RobotMap.g1.dpad_up)
            isArcadeDrive = false;
        else if(RobotMap.g1.dpad_down)
            isArcadeDrive = true;
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
        double[] values = new double[motors.length];
        for(int i = 0; i < motors.length; i ++)
            values[i] = motors[i].getPower();
        return values;
    }

    @Override
    public void setValues(double[] vals) {
        for(int i = 0; i < vals.length; i ++)
            motors[i].setPower(vals[i]);
    }
}
