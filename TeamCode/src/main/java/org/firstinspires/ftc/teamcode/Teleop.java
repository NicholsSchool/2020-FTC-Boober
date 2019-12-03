package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;


@TeleOp(name="BooberTeleop", group="Iterative Opmode")
public class Teleop extends OpMode
{
    @Override
    public void init() {
        Robot.init(hardwareMap, FtcDashboard.getInstance().getTelemetry(), gamepad1, gamepad2);

    }

    @Override
    public void loop() {
        Robot.run();
        if(RobotMap.g1.y) {
            Robot.camera.testDetector();
            RobotMap.telemetry.update();
        }
//        colorSensorData(RobotMap.lColorSensor, "left");
//        colorSensorData(RobotMap.rColorSensor, "right");

    }

    private void colorSensorData(ColorSensor sensor, String type)
    {
        RobotMap.telemetry.addData(type + "sensor blue",sensor.blue());
        RobotMap.telemetry.addData(type + "sensor green",sensor.green());
        RobotMap.telemetry.addData(type + "sensor red",sensor.red());
        RobotMap.telemetry.addData(type + "sensor alpha",sensor.alpha());
        RobotMap.telemetry.addData(type + "sensor alpha",sensor.argb());
    }

    @Override
    public void stop() {
        Robot.stop();
    }
}
