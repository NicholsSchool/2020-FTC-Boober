package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
/**
 * All possible values that may need adjusting for robot gameplay
 */
public class Constants {

    public static double DRIVE_BUFFER = 1.0;
    public static double SLOW_INTAKE_SPEED = 0.5;
    public static double FAST_INTAKE_SPEED = 1;
    public static double FAST_OUTTAKE_SPEED = -1.0;
    public static double SLOW_OUTTAKE_SPEED = -0.5;

    public static boolean IN_TEST_MODE = true;
    public static double TEST_TURN_SPEED = 0.5, TEST_TURN_ANGLE = 90;
    public static int ROBOT_START_POSITION = 1;
    public static int TEST_SKYSTONE_POSITION = 1;
}
