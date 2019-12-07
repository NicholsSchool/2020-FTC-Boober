package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;
import org.firstinspires.ftc.teamcode.util.pid.PIDController;
import org.firstinspires.ftc.teamcode.util.record.Recordable;

import java.util.function.Consumer;

public class DriveTrain extends Subsystem implements Recordable {


    private static final double
            COUNTS_PER_REV  = -530, INCHES_PER_REV = 4 * Math.PI, COUNTS_PER_INCH = COUNTS_PER_REV/INCHES_PER_REV;
    private DcMotorSimple[] motors;
    private boolean isArcadeDrive;
    /**
     * Constructs a new DriveTrain subsystem object
     * @param name - the name for the Subsystem
     */
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

        isArcadeDrive = false;
    }

    /**
     * Moves the robot's drivetrain along an x^2 curve
     * @param lSpeed - the speed for the left side of the robot
     * @param rSpeed - the speed for the right side of the robot
     */
    public void move(double lSpeed, double rSpeed)
    {
        lSpeed = Range.clip(lSpeed, -1, 1);
        rSpeed = Range.clip(rSpeed, -1, 1);

        // The x^2 movement Dillan wanted.
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

    /**
     * Runs tank drive control for driver
     */
    public void tankDrive()
    {
        move(RobotMap.g1.left_stick_y, RobotMap.g1.right_stick_y);
    }

    /**
     * Runs arcade drive control for driver
     */
    public void arcadeDrive()
    {
        move( RobotMap.g1.right_stick_y - RobotMap.g1.left_stick_x, RobotMap.g1.right_stick_y + RobotMap.g1.left_stick_x);
    }

    /**
     * <u>Auto Method</u>
     * Uses the built in PID encoder movements of the DcMotor class to drive to position
     * @param speed - the speed to drive at
     * @param leftInches - the number of inches for the left side to move
     * @param rightInches - the number of inches for the right side to move
     */
    public void encoderDrive(double speed, double leftInches, double rightInches) {

            int[] targets = new int[4];
            int count = 0;
            int error = 10;
            for(int i = 0; i < motors.length; i ++)
            {
                if(!(motors[i] instanceof  DcMotor))
                    continue;
                DcMotor m = (DcMotor) motors[i];
                int target = m.getCurrentPosition() + (int)((i < motors.length/2 ? leftInches : rightInches) * COUNTS_PER_INCH);
                targets[count ++ ] = target;
                m.setTargetPosition(target);
                m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                m.setPower(speed);
            }

            boolean leftIsBusy = true, rightIsBusy = true;
            boolean prettyMuchThere = false;
            while (Robot.opMode.opModeIsActive() && (leftIsBusy || rightIsBusy) && !prettyMuchThere) {
                count = 0;
                prettyMuchThere = true;

                for(int i = 0; i < motors.length; i ++)
                {
                    if(!(motors[i] instanceof  DcMotor)) {
                     //   motors[i].setPower(motors[i + 1].getPower());
                        continue;
                    }
                    DcMotor m = (DcMotor) motors[i];
                    prettyMuchThere = prettyMuchThere && Math.abs(m.getCurrentPosition() - targets[count ++]) < error;
                    if(i < motors.length)
                        leftIsBusy = leftIsBusy && m.isBusy();
                    else
                        rightIsBusy = rightIsBusy && m.isBusy();
                }
            }

            for(int i = 0; i < motors.length; i ++)
            {
                if(!(motors[i] instanceof  DcMotor)) {
                  //  motors[i].setPower(0);
                    continue;
                }
                DcMotor m = (DcMotor) motors[i];
                m.setPower(0);
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Check what mode it should be
            }



    }

    /**
     * Turns on or off Brake ZeroPowerBehavior for the DcMotors within the DriveTrain
     * @param turnOn - true if Brake ZeroPowerBehavior is desired
     */
    public void setBrakeMode(boolean turnOn)
    {
        DcMotor.ZeroPowerBehavior type =  DcMotor.ZeroPowerBehavior.BRAKE;
        if(!turnOn)
        {
            type = DcMotor.ZeroPowerBehavior.FLOAT;
        }
        for(int i = 0; i < motors.length; i ++)
        {
            if(!(motors[i] instanceof  DcMotor)) {
                continue;
            }
            DcMotor m = (DcMotor) motors[i];
            m.setZeroPowerBehavior(type);
        }
    }

    public void testMove(double leftSpeed, double rightSpeed, double angle)
    {
        Robot.gyro.resetAngle();
        move(leftSpeed, rightSpeed);
        boolean turn = true;
        while(Robot.opMode.opModeIsActive() && turn)
        {
            if(angle > 0)
                turn = Robot.gyro.getAngle() < angle;
            else
                turn = Robot.gyro.getAngle() > angle;
            Robot.gyro.print();
            System.out.println("Gyro Angle: " + Robot.gyro.getAngle());
            System.out.println("Going to: " + angle);
            RobotMap.telemetry.update();
        }
        stop();
        resetEncoders();
        System.out.println("\n\n\n");
    }

    /**
     * <u>Auto Method</u>
     * Does not use DcMotor's built in PID encoder movements but still uses the encoder values
     * (hence the name <b>half</b> Encoder Drive)
     * @param speed - the speed to drive at
     * @param leftInches - the number of inches for the left side to move
     * @param rightInches - the number of inches for the right side to move
     */
    public void halfEncoderDrive(double speed, double leftInches, double rightInches)
    {
        speed *= -1;
        int[] targets = new int[4];
        int count = 0;
        int error = 15;
        for(int i = 0; i < motors.length; i ++)
        {
            if(!(motors[i] instanceof  DcMotor))
                continue;
            DcMotor m = (DcMotor) motors[i];
            int target = m.getCurrentPosition() + (int)((i < motors.length/2 ? leftInches : rightInches) * COUNTS_PER_INCH);
            targets[count ++ ] = target;
        }

        boolean prettyMuchThere = false;
        while (Robot.opMode.opModeIsActive()  && !prettyMuchThere) {
            count = 0;
            double average = 0;
            for(int i = 0; i < motors.length; i ++)
            {
                if(!(motors[i] instanceof  DcMotor)) {
                       motors[i].setPower(speed);
                        continue;
                }
                DcMotor m = (DcMotor) motors[i];
                average +=  targets[count ++] - m.getCurrentPosition();
            }
            System.out.println(average/(double)count);
            if(speed > 0)
                prettyMuchThere = average/(double)count < error;
            else
                prettyMuchThere = average/(double)count > -error;
            RobotMap.telemetry.update();

        }
        for(int i = 0; i < motors.length; i ++) {
            if (!(motors[i] instanceof DcMotor)) {
                motors[i].setPower(0);
                continue;
                // resetEncoders();

            }
        }
    }

    /**
     * <u>Auto Method</u>
     * Turns to angle and self-corrects based on PID
     * <i>Currently In Testing Phase (Not Working)</i>
     * @param speed - the max speed to move at
     * @param angle - the target angle
     */
    public void PIDTurn(double speed, double angle)
    {
        Robot.gyro.resetAngle();
        PIDController turnControl = new PIDController(0.003, 0, 0);
        turnControl.reset();
        turnControl.setInputRange(0, angle);
        turnControl.setOutputRange(0, speed);
        turnControl.setTolerance(1);
        turnControl.enable();

        double negation = 1;
        if(angle < 0)
            negation = -1;
        RobotMap.telemetry.addData("PID TURN",turnControl.onTarget());
        while(Robot.opMode.opModeIsActive() && !turnControl.onTarget())
        {
            double power = turnControl.performPID(Robot.gyro.getAngle());
            move(negation * power, -negation * power);
            Robot.gyro.print();
            RobotMap.telemetry.update();
        }
        stop();
    }

    /**
     * <u>Auto Method</u>
     * Turns Robot to given angle
     * @param speed - the speed to turn
     * @param angle - the target angle
     */
    public void turn(double speed, double angle)
    {
        Robot.gyro.resetAngle();
          // turn right.
        double leftPower = speed;
        double rightPower = -speed;

        if (angle < 0)
        {   // turn left.
            leftPower = -speed;
            rightPower = speed;
        }

        move(leftPower, rightPower);
        boolean turn = true;
        while(Robot.opMode.opModeIsActive() && turn)
        {
            if(angle > 0)
                turn = Robot.gyro.getAngle() < angle;
            else
                turn = Robot.gyro.getAngle() > angle;
            Robot.gyro.print();
            System.out.println("Gyro Angle: " + Robot.gyro.getAngle());
            System.out.println("Going to: " + angle);
            RobotMap.telemetry.update();
        }
        stop();
        resetEncoders();
        System.out.println("\n\n\n");
    }

    /**
     * <u>Auto Method</u>
     * Uses gyro instance to drive in a straight line
     * <i>Currently In Testing Phase (Not Working)</i>
     * @param speed - the speed to drive
     * @param time - the amount of time to drive for
     */
    public void gyroStraight(double speed, double time)
    {
        Robot.gyro.resetAngle();
        RobotMap.timer.reset();
        while(Robot.opMode.opModeIsActive() &&  RobotMap.timer.time() < time )
        {
            double correction = Robot.gyro.getCorrection();
            move(speed - correction, speed + correction);
        }
        stop();
    }

    /**
     * <u>Auto Method</u>
     * Drive until the Colored tape is seen
     * <i>Currently In Testing Phase (Not Working)</i>
     * @param speed - the speed to drive
     */
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

    /**
     * Prints the encoder values of the DcMotors
     * <u>Used for Testing</u>
     */
    public void printEncoders(){
        for(int i = 0; i < motors.length; i ++)
        {
            if(!(motors[i] instanceof  DcMotor))
                continue;
            DcMotor m = (DcMotor) motors[i];
            RobotMap.telemetry.addData("Encoder i ", m.getCurrentPosition());
        }
    }

    /**
     * Resets all encoders
     */
    public void resetEncoders()
    {

        for(int i = 0; i < motors.length; i ++)
        {
            if(!(motors[i] instanceof  DcMotor))
                continue;
            DcMotor m = (DcMotor) motors[i];
            m.setPower(0);
            DcMotor.RunMode mode = m.getMode();
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(mode);
        }
    }

    @Override
    /**
     * DriveTrain's gamepad control code for teleop
     */
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
    }

    @Override
    /**
     * Stops all DriveTrain movement
     */
    public void stop()
    {
        move(0,0);
    }

    @Override
    /**
     * Returns values to records
     */
    public double[] getValues() {
        double[] values = new double[motors.length];
        for(int i = 0; i < motors.length; i ++)
            values[i] = motors[i].getPower();
        return values;
    }

    @Override
    /**
     * Sets DriveTrain's motors to given values
     */
    public void setValues(double[] vals) {
        for(int i = 0; i < vals.length; i ++)
            motors[i].setPower(vals[i]);
    }
}
