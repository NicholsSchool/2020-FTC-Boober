package org.firstinspires.ftc.teamcode.util;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.sensors.*;

import java.util.HashMap;

/**
 * Used to store and run all Subsystems and Sensors within the Robot
 */
public class Robot {

    public static DriveTrain driveTrain;
    public static Intake intake;
    public static Elevator elevator;
    public static Claw claw;
    public static Camera camera;
    public static Gyro gyro;
    public static DistanceSensor distanceSensor;
    public static LinearOpMode opMode;
    public static ColorPicker colorPicker;
    public static String filePath;
    public static String fileName;

    private static HashMap<String, Subsystem> subsystems;

    /**
     * Instantiates the RobotMap and all our Subsystem and Sensor classes used by the Robot
     * @param hw - the HardwareMap created by the OpMode
     * @param t - the Telemetry created by the OpMode
     * @param g1 - The gamepad1 created by the OpMode
     * @param g2 - The gamepad2 created by the OpMode
     */
    public static void init(HardwareMap hw, Telemetry t, Gamepad g1, Gamepad g2)
    {
        RobotMap.init(hw, t, g1, g2);
        driveTrain = new DriveTrain("DriveTrain");
        intake = new Intake("Intake");
        elevator = new Elevator("Elevator");
        claw = new Claw("Claw");
        camera = new Camera();
        gyro = new Gyro();
        colorPicker = new ColorPicker();
        distanceSensor = new DistanceSensor();
        filePath = Environment.getExternalStorageDirectory().getPath();
        fileName = "SimpleBuildSide";
    }

    /**
     * Instantiates the RobotMap and all our Subsystem and Sensor classes used by the Robot, along
     * with storing the LinearOpMode for autos
     * @param hw - the HardwareMap created by the OpMode
     * @param t - the Telemetry created by the OpMode
     * @param g1 - The gamepad1 created by the OpMode
     * @param g2 - The gamepad2 created by the OpMode
     * @param mode - the LinearOpMode for autos
     */
    public static void init(HardwareMap hw, Telemetry t, Gamepad g1, Gamepad g2, LinearOpMode mode)
    {
        init(hw, t, g1, g2);
        opMode = mode;
    }

    /**
     * Puts the inputted Subsystem in the storage HashMap, with the subsystem's name as the key.
     * @param s - the Subsystem to add to the storage HashMap
     */
    public static void registerSubsystem(Subsystem s)
    {
        if(subsystems == null)
            subsystems = new HashMap<>();
        subsystems.put(s.getName(), s);
    }

    /**
     * Returns the storage HashMap of Subsystems
     * @return the storage HashMap of Subsystems
     */
    public static HashMap<String, Subsystem> getSubsystems()
    {
        return subsystems;
    }

    /**
     * Loops through all Subsystems and runs them
     */
    public static void run()
    {
        for(Subsystem s : subsystems.values())
            s.run();
    }

    /**
     * Loops through all Subsystems and stops them
     */
    public static void stop()
    {
        for(Subsystem s : subsystems.values())
            s.stop();
    }

}
