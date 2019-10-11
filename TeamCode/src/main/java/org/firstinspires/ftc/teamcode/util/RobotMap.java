package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotMap {
    public static DcMotor lfDrive, lmDrive, lbDrive, rfDrive, rmDrive, rbDrive;

    public static void init(HardwareMap hw)
    {
        lfDrive = hw.get(DcMotor.class, "LFDrive");
        lmDrive = hw.get(DcMotor.class, "LMDrive");
        lbDrive = hw.get(DcMotor.class, "LBDrive");

        rfDrive = hw.get(DcMotor.class, "RFDrive");
        rmDrive = hw.get(DcMotor.class, "RMDrive");
        rbDrive = hw.get(DcMotor.class, "RBDrive");

        lfDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        lmDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        lbDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        rfDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rmDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rbDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}
