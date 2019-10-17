package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotMap {
    public static DcMotor lfDrive, lmDrive, lbDrive, rfDrive, rmDrive, rbDrive;
    public static Telemetry telemetry;
    public static Gamepad g1, g2;

    public static void init(HardwareMap hw, Telemetry t, Gamepad gamepad1, Gamepad gamepad2)
    {
        lfDrive = hw.get(DcMotor.class, "LFDrive");
        lmDrive = hw.get(DcMotor.class, "LMDrive");
        lbDrive = hw.get(DcMotor.class, "LBDrive");

        rfDrive = hw.get(DcMotor.class, "RFDrive");
        rmDrive = hw.get(DcMotor.class, "RMDrive");
        rbDrive = hw.get(DcMotor.class, "RBDrive");

        telemetry = t;

        g1 = gamepad1;
        g2 = gamepad2;

        lfDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        lmDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        lbDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        rfDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rmDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rbDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}
