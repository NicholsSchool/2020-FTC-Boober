package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;


@TeleOp(name="Ok Boober Teleop", group="Iterative Opmode")
/**
 * The game play teleop code to run
 */
public class Teleop extends OpMode
{
    @Override
    /**
     * Intializes the objects within the Robot class
     */
    public void init() {
        Robot.init(hardwareMap, FtcDashboard.getInstance().getTelemetry(), gamepad1, gamepad2);
    }

    @Override
    /**
     * Runs the robot movements
     */
    public void loop() {
        Robot.run();
    }

    @Override
    /**
     * Stops all Robot movements
     */
    public void stop() {
        Robot.stop();
    }
}
