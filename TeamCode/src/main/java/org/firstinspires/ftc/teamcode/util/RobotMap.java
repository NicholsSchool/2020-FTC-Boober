package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This class houses all motors and sensors used by the object. This class exists so that we can avoid
 * having to use the same code repeatedly in each of our OpModes. Also, if a change is needed to the instantiate
 * of a variable, the singular change done in this class will be applied to all our OpModes.
 */
public class RobotMap {
    public static DcMotor lmDrive, lbDrive, rmDrive, rbDrive;
    public static DcMotorSimple lfDrive, rfDrive;
    public static DcMotorSimple lGripper, rGripper;
    public static CRServo elevator;
    public static Rev2mDistanceSensor backDistanceSensor, frontDistanceSensor;
    public static ColorSensor rColorSensor, lColorSensor;
    public static AnalogInput colorDial;
    public static CRServo claw;
    public static Telemetry telemetry;
    public static    BNO055IMU imu;
    public static Gamepad g1, g2;
    public static ElapsedTime timer;


    /**
     * This method will instantiate all motors and sensors used by the robot.
     * @param hw - the HardwareMap created by the OpMode
     * @param t - the Telemetry created by the OpMode
     * @param gamepad1 - The gamepad1 created by the OpMode
     * @param gamepad2 - The gamepad2 created by the OpMode
     */
    public static void init(HardwareMap hw, Telemetry t, Gamepad gamepad1, Gamepad gamepad2)
    {
        lfDrive = hw.get(DcMotorSimple.class, "LFDrive");
        lmDrive = hw.get(DcMotorImplEx.class, "LMDrive");
        lbDrive = hw.get(DcMotorImplEx.class, "LBDrive");

        rfDrive = hw.get(DcMotorSimple.class, "RFDrive");
        rmDrive = hw.get(DcMotorImplEx.class, "RMDrive");
        rbDrive = hw.get(DcMotorImplEx.class, "RBDrive");

        lGripper = hw.get(DcMotorSimple.class, "LGripper");
        rGripper = hw.get(DcMotorSimple.class, "RGripper");

        elevator = hw.get(CRServo.class, "Elevator");

        claw = hw.get(CRServo.class, "Claw");

        backDistanceSensor = hw.get(Rev2mDistanceSensor.class, "DistanceSensor");
        frontDistanceSensor = hw.get(Rev2mDistanceSensor.class, "FrontDistanceSensor");
        rColorSensor = hw.get(ColorSensor.class, "RColorSensor");
        lColorSensor = hw.get(ColorSensor.class, "LColorSensor");

        colorDial = hw.get(AnalogInput.class, "ColorDial");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hw.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        timer = new ElapsedTime();

        telemetry = t;

        g1 = gamepad1;
        g2 = gamepad2;


        lGripper.setDirection(DcMotorSimple.Direction.REVERSE);
        rGripper.setDirection(DcMotorSimple.Direction.REVERSE);

        elevator.setDirection(DcMotorSimple.Direction.REVERSE);

        lfDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        lmDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        lbDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        rfDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rmDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rbDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        claw.setDirection(DcMotorSimple.Direction.REVERSE);

    }
}
