package org.firstinspires.ftc.teamcode.util;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.sensors.*;

import java.util.HashMap;

public class Robot {

    public static DriveTrain driveTrain;
    public static Intake intake;
    public static Elevator elevator;
    public static Claw claw;
    public static Camera camera;
    public static LinearOpMode opMode;
    public static String filePath;

    private static HashMap<String, Subsystem> subsystems;

    public static void init(HardwareMap hw, Telemetry t, Gamepad g1, Gamepad g2)
    {
        RobotMap.init(hw, t, g1, g2);
        driveTrain = new DriveTrain("DriveTrain");
        intake = new Intake("Intake");
        elevator = new Elevator("Elevator");
        claw = new Claw("Claw");
        camera = new Camera();
        filePath = Environment.getExternalStorageDirectory().getPath();
    }

    public static void init(HardwareMap hw, Telemetry t, Gamepad g1, Gamepad g2, LinearOpMode mode)
    {
        init(hw, t, g1, g2);
        opMode = mode;
    }


    public static void registerSubsystem(Subsystem s)
    {
        if(subsystems == null)
            subsystems = new HashMap<>();
        subsystems.put(s.getName(), s);
    }

    public HashMap<String, Subsystem> getSubsystems()
    {
        return subsystems;
    }

    public static void run()
    {
        for(Subsystem s : subsystems.values())
            s.run();
    }

    public static void stop()
    {
        for(Subsystem s : subsystems.values())
            s.stop();
    }

}
