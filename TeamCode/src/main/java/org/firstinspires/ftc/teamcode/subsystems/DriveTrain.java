package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.RobotMap;

public class DriveTrain {

    private DcMotor[] motors;

    public DriveTrain()
    {
        motors = new DcMotor[6];
        motors[0] = RobotMap.lfDrive;
        motors[1] = RobotMap.lmDrive;
        motors[2] = RobotMap.lbDrive;

        motors[3] = RobotMap.rfDrive;
        motors[4] = RobotMap.rmDrive;
        motors[5] = RobotMap.rbDrive;



    }

    public void move(double lSpeed, double rSpeed)
    {
        if(Math.abs(lSpeed - rSpeed) > 1.0)
        {
            lSpeed *= 0.5;
            rSpeed *= 0.5;
        }

        for(int i = 0; i < motors.length; i ++)
        {
            if(i < motors.length/2)
                motors[i].setPower(lSpeed);
            else
                motors[i].setPower(rSpeed);

        }

    }

    public void encoderTest()
    {
        for(int i = 0;  i < motors.length; i ++)

            RobotMap.telemetry.addData("Encoder "  + i , motors[i].getCurrentPosition());
        RobotMap.telemetry.update();
    }

    public void stop()
    {
        move(0,0);
    }
}
