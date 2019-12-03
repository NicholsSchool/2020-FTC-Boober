package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotMap {
    public static DcMotor lmDrive, lbDrive, rmDrive, rbDrive;
    public static DcMotorSimple lfDrive, rfDrive;
    public static DcMotorSimple lGripper, rGripper;
    public static CRServo elevator;
    public static TouchSensor intakeTouchSensor;
    public static ColorSensor rColorSensor, lColorSensor;
    public static CRServo claw;
    public static Telemetry telemetry;
    public static Gamepad g1, g2;

    public static void init(HardwareMap hw, Telemetry t, Gamepad gamepad1, Gamepad gamepad2)
    {
        lfDrive = hw.get(DcMotorSimple.class, "LFDrive");
        lmDrive = hw.get(DcMotor.class, "LMDrive");
        lbDrive = hw.get(DcMotor.class, "LBDrive");

        rfDrive = hw.get(DcMotorSimple.class, "RFDrive");
        rmDrive = hw.get(DcMotor.class, "RMDrive");
        rbDrive = hw.get(DcMotor.class, "RBDrive");

        lGripper = hw.get(DcMotorSimple.class, "LGripper");
        rGripper = hw.get(DcMotorSimple.class, "RGripper");

        elevator = hw.get(CRServo.class, "Elevator");

        claw = hw.get(CRServo.class, "Claw");

        intakeTouchSensor = hw.get(TouchSensor.class,"TouchSensor");
        rColorSensor = hw.get(ColorSensor.class, "RColorSensor");
        lColorSensor = hw.get(ColorSensor.class, "LColorSensor");

        telemetry = t;

        g1 = gamepad1;
        g2 = gamepad2;

        lmDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rmDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lGripper.setDirection(DcMotorSimple.Direction.REVERSE);
        rGripper.setDirection(DcMotorSimple.Direction.REVERSE);

        elevator.setDirection(DcMotorSimple.Direction.REVERSE);

        lfDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        lmDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        lbDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        rfDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rmDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rbDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        claw.setDirection(DcMotorSimple.Direction.REVERSE);

    }
}
