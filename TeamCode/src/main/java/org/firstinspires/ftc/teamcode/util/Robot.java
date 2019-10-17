package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

public class Robot {

    public static DriveTrain driveTrain;

    public static void init(HardwareMap hw, Telemetry t, Gamepad g1, Gamepad g2)
    {
        RobotMap.init(hw, t, g1, g2);
        driveTrain = new DriveTrain();
    }
}
