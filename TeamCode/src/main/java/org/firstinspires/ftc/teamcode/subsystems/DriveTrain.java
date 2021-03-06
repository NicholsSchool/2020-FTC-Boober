package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.autonomous.Function;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;
import org.firstinspires.ftc.teamcode.util.record.Recordable;


public class DriveTrain extends Subsystem implements Recordable {


    private static final double
            COUNTS_PER_REV  = 515, INCHES_PER_REV = 4 * Math.PI, COUNTS_PER_INCH = COUNTS_PER_REV/INCHES_PER_REV;
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
    public void move(double lSpeed, double rSpeed) {
        move(lSpeed, rSpeed, true);
    }

    private void move(double lSpeed, double rSpeed, boolean squareMovement)
    {
        lSpeed = Range.clip(lSpeed, -1, 1);
        rSpeed = Range.clip(rSpeed, -1, 1);

        // The x^2 movement Dillan wanted.
        if(squareMovement) {
            lSpeed *= Math.abs(lSpeed);
            rSpeed *= Math.abs(rSpeed);
        }

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
        move(-RobotMap.g1.left_stick_y, -RobotMap.g1.right_stick_y);
    }

    /**
     * Runs arcade drive control for driver
     */
    public void arcadeDrive()
    {
        move( -(RobotMap.g1.right_stick_y - RobotMap.g1.left_stick_x), -(RobotMap.g1.right_stick_y + RobotMap.g1.left_stick_x));
    }

    /**
     * <u>Auto Method</u>
     * Uses the built in PID encoder movements of the DcMotor class to drive to position
     * @param speed - the speed to drive at
     * @param leftInches - the number of inches for the left side to move
     * @param rightInches - the number of inches for the right side to move
     * @param timeoutS - max seconds for the method to run
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS)
    {
        encoderDrive(speed, leftInches, rightInches, timeoutS, new Function() {
            public void execute(){}
            public void stop(){}
        });
    }

    /**
     * <u>Auto Method</u>
     * Uses the built in PID encoder movements of the DcMotor class to drive to position
     * @param speed - the speed to drive at
     * @param leftInches - the number of inches for the left side to move
     * @param rightInches - the number of inches for the right side to move
     * @param timeoutS - max seconds for the method to run
     * @param f - another robot functionality to run while this loop runs
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS, Function f)
    {
        int lmTarget, lbTarget, rmTarget, rbTarget;

        // Ensure that the opmode is still active
        if (Robot.opMode.opModeIsActive()) {
            resetEncoders();
            // Determine new target position, and pass to motor controller
            lmTarget = RobotMap.lmDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            lbTarget = RobotMap.lbDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            rmTarget = RobotMap.rmDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            rbTarget = RobotMap.rbDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            RobotMap.lmDrive.setTargetPosition(lmTarget);
            RobotMap.lbDrive.setTargetPosition(lbTarget);
            RobotMap.rmDrive.setTargetPosition(rmTarget);
            RobotMap.rbDrive.setTargetPosition(rbTarget);

            // Turn On RUN_TO_POSITION
            RobotMap.lmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RobotMap.lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            RobotMap.rmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RobotMap.rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            RobotMap.timer.reset();
            RobotMap.lmDrive.setPower(Math.abs(speed));
            RobotMap.lbDrive.setPower(Math.abs(speed));

            RobotMap.rmDrive.setPower(Math.abs(speed));
            RobotMap.rbDrive.setPower(Math.abs(speed));

            //Note: Possibly to make this better, make each side &&
            while (Robot.opMode.opModeIsActive() && (RobotMap.timer.seconds() < timeoutS) &&
                    ((RobotMap.lmDrive.isBusy() || RobotMap.lbDrive.isBusy()) &&
                            (RobotMap.rmDrive.isBusy() || RobotMap.rbDrive.isBusy()))) {
                f.execute();
            }
            f.stop();
            // Stop all motion;
            RobotMap.lmDrive.setPower(Math.abs(0));
            RobotMap.lbDrive.setPower(Math.abs(0));

            RobotMap.rmDrive.setPower(Math.abs(0));
            RobotMap.rbDrive.setPower(Math.abs(0));

            // Turn off RUN_TO_POSITION
            RobotMap.lmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RobotMap.lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            RobotMap.rmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RobotMap.rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    //The difference between this and encoderDrive is that this one keeps running until both sides of drivetrain
    //have finished moving, not just one
    public void firmEncoderDrive(double speed, double leftInches, double rightInches, double timeoutS, Function f)
    {
        int lmTarget, lbTarget, rmTarget, rbTarget;

        // Ensure that the opmode is still active
        if (Robot.opMode.opModeIsActive()) {
            resetEncoders();
            // Determine new target position, and pass to motor controller
            lmTarget = RobotMap.lmDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            lbTarget = RobotMap.lbDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            rmTarget = RobotMap.rmDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            rbTarget = RobotMap.rbDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            RobotMap.lmDrive.setTargetPosition(lmTarget);
            RobotMap.lbDrive.setTargetPosition(lbTarget);
            RobotMap.rmDrive.setTargetPosition(rmTarget);
            RobotMap.rbDrive.setTargetPosition(rbTarget);

            // Turn On RUN_TO_POSITION
            RobotMap.lmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RobotMap.lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            RobotMap.rmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RobotMap.rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            RobotMap.timer.reset();
            RobotMap.lmDrive.setPower(Math.abs(speed));
            RobotMap.lbDrive.setPower(Math.abs(speed));

            RobotMap.rmDrive.setPower(Math.abs(speed));
            RobotMap.rbDrive.setPower(Math.abs(speed));

            // the or after the lbdrive, is the difference between this and encoderDrive
            while (Robot.opMode.opModeIsActive() && (RobotMap.timer.seconds() < timeoutS) &&
                    ((RobotMap.lmDrive.isBusy() || RobotMap.lbDrive.isBusy()) ||
                            (RobotMap.rmDrive.isBusy() || RobotMap.rbDrive.isBusy()))) {

                f.execute();
                printEncodersInInches();
                printEncoders();
                RobotMap.telemetry.update();
                Robot.gyro.testPrint();
            }
            f.stop();
            // Stop all motion;
            RobotMap.lmDrive.setPower(Math.abs(0));
            RobotMap.lbDrive.setPower(Math.abs(0));

            RobotMap.rmDrive.setPower(Math.abs(0));
            RobotMap.rbDrive.setPower(Math.abs(0));

            // Turn off RUN_TO_POSITION
            RobotMap.lmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RobotMap.lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            RobotMap.rmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RobotMap.rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Robot.gyro.testPrint();
        }
    }

    /**
     * <u>Auto Method</u>
     * Method tries to have the robot drive and turn at the same time, by running the sides at different speeds
     * @param leftSpeed - the speed for the left side
     * @param rightSpeed - the speed for the right side
     * @param desiredHeading - the heading to finish turning at
     * @param timeoutS - the max amount of time to run the loop
     * @param f - another robot functionality to run while this loop runs
     */
    public void driveAndTurn(double leftSpeed, double rightSpeed, double desiredHeading, double timeoutS, Function f)
    {
        if(Robot.opMode.opModeIsActive()) {
            RobotMap.lfDrive.setPower(leftSpeed);
            RobotMap.lmDrive.setPower(leftSpeed);
            RobotMap.rfDrive.setPower(rightSpeed);
            RobotMap.rbDrive.setPower(rightSpeed);
            RobotMap.timer.reset();
            boolean turn = true;
            double error = (Robot.gyro.getHeading() - desiredHeading);
            while (Robot.opMode.opModeIsActive() && Math.abs(error) > 0.3  && (RobotMap.timer.time() < timeoutS)) {

                Robot.gyro.print();
                RobotMap.telemetry.update();
                f.execute();
            }
            f.stop();
            stop();
            resetEncoders();
            System.out.println("\n\n\n");
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
            if(!(motors[i] instanceof  DcMotorImplEx)) {
                continue;
            }
            DcMotorImplEx m = (DcMotorImplEx) motors[i];
            m.setZeroPowerBehavior(type);
        }
    }

    /**
     * <u>Auto Method</u>
     * Turns Robot to a given heading using PID control
     * @param inputSpeed - the speed to turn
     * @param desiredHeading - the target heading
     * @param timeoutS - the max amount of time to run method
     */
    public void turnOnHeading(double inputSpeed, double desiredHeading, double timeoutS)
    {
        turnOnHeading(inputSpeed, desiredHeading, timeoutS, new Function() {
            @Override
            public void execute() {

            }
            @Override
            public void stop() {

            }
        });
    }

    /**
     * <u>Auto Method</u>
     * Turns Robot to a given heading using PID control
     * @param inputSpeed - the speed to turn
     * @param desiredHeading - the target heading
     * @param timeoutS - the max amount of time to run method
     * @param f - another robot functionality to run while this loop runs
     */
    public void turnOnHeading(double inputSpeed, double desiredHeading, double timeoutS, Function f)
    {
        if(Robot.opMode.opModeIsActive()) {

            double negation = 1; //turn right
            if(desiredHeading - Robot.gyro.getHeading() > 0) // Check this for correctness
                negation = -1; // turn left

            RobotMap.timer.reset();


            double p = Constants.TURN_P, i = Constants.TURN_I, d= Constants.TURN_D;
            double minSpeed = 0.3,
                    error = negation * (Robot.gyro.getHeading() - desiredHeading),
                    prevError = error,
                    sumError = 0;
            while (Robot.opMode.opModeIsActive() &&  Math.abs(error) > 0.3 && (RobotMap.timer.time() < timeoutS)) {
                double currentAngle = Robot.gyro.getHeading();
                error = negation * (currentAngle - desiredHeading);

                double newSpeed = p * error + i * sumError + d * (error - prevError);
                newSpeed = Range.clip(newSpeed, -1, 1 ) ;
                prevError = error;
                if(Math.abs(sumError + error) * i < 1)
                    sumError += error;

                double finalSpeed = negation * (newSpeed) ;

                move(finalSpeed, -finalSpeed, false);
                f.execute();
                RobotMap.telemetry.addData("Turn P", p);
                RobotMap.telemetry.addData("Turn I", i);
                RobotMap.telemetry.addData("Turn D", d);
                RobotMap.telemetry.addLine("\n");
                RobotMap.telemetry.addData("Error", error);
                RobotMap.telemetry.update();
            }
            f.stop();
            stop();
        }
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
            RobotMap.telemetry.addData("Encoder " + i, m.getCurrentPosition());
        }
    }

    public void printEncodersInInches()
    {
        for(int i = 0; i < motors.length; i ++)
        {
            if(!(motors[i] instanceof  DcMotor))
                continue;
            DcMotor m = (DcMotor) motors[i];
            RobotMap.telemetry.addData("Inches Encoder " + i, m.getCurrentPosition()/COUNTS_PER_INCH);
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

    /**
     * DriveTrain's gamepad control code for teleop
     */
    @Override
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

    /**
     * Stops all DriveTrain movement
     */
    @Override
    public void stop()
    {
        move(0,0);
    }

    /**
     * Returns values to records
     */
    @Override
    public double[] getValues() {
        double[] values = new double[motors.length];
        for(int i = 0; i < motors.length; i ++)
            values[i] = motors[i].getPower();
        return values;
    }

    /**
     * Sets DriveTrain's motors to given values
     */
    @Override
    public void setValues(double[] vals) {
        for(int i = 0; i < vals.length; i ++)
            motors[i].setPower(vals[i]);
    }
}
