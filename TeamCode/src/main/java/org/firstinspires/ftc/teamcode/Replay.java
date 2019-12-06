package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.record.RecordReader;

import java.io.FileNotFoundException;

@Disabled
@TeleOp(name="Replay Test", group="Recording Tests")
public class Replay extends OpMode {

    private RecordReader reader;
    @Override
    /**
     * Intializes the objects within the Robot class and the RecordReader instance
     */
    public void init() {

        Robot.init(hardwareMap, telemetry, gamepad1, gamepad2);

        try {
            reader = new RecordReader(Robot.filePath, Robot.fileName);
        } catch (FileNotFoundException e) {
            telemetry.addData("Init File not found Error", e);
            telemetry.update();
        }

    }

    @Override
    /**
     * Has the reader read through the selected replay file
     */
    public void loop() {
        if(reader.isReading())
            reader.read();
    }

    @Override
    /**
     * Stops reading and stops the robot
     */
    public void stop()
    {
        if(reader != null)
            reader.stop();
        Robot.stop();
    }
}
