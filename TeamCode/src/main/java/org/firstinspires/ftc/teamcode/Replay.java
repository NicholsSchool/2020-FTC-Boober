package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.record.RecordReader;

import java.io.FileNotFoundException;


@TeleOp(name="Replay Test", group="Recording Tests")
public class Replay extends OpMode {

    private RecordReader reader;
    /**
     * Intializes the objects within the Robot class and the RecordReader instance
     */
    @Override
    public void init() {

        Robot.init(hardwareMap, telemetry, gamepad1, gamepad2);

        try {
            reader = new RecordReader(Robot.filePath, Robot.fileName);
        } catch (FileNotFoundException e) {
            telemetry.addData("Init File not found Error", e);
            telemetry.update();
        }

    }

    /**
     * Has the reader read through the selected replay file
     */
    @Override
    public void loop() {
        if(reader.isReading())
            reader.read();
    }

    /**
     * Stops reading and stops the robot
     */
    @Override
    public void stop()
    {
        if(reader != null)
            reader.stop();
        Robot.stop();
    }
}
