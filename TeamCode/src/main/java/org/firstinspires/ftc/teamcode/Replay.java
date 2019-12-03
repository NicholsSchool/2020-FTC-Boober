package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.record.RecordReader;

import java.io.FileNotFoundException;

@TeleOp(name="Replay Test", group="Recording Tests")
public class Replay extends OpMode {

    RecordReader reader;
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

    @Override
    public void loop() {
        if(reader.isReading())
            reader.read();
//        telemetry.addData("isReading", reader.isReading());
//        telemetry.update();
    }

    @Override
    public void stop()
    {
        reader.stop();
        Robot.stop();
    }
}
