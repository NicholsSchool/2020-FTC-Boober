package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.record.RobotRecorder;

import java.io.IOException;

@Disabled
@TeleOp(name="Teleop Record Test", group="Recording Tests")
public class Record extends OpMode {
    private RobotRecorder recorder;
    private boolean isRecording;

    /**
     * Intializes the objects within the Robot class and the RobotRecorder instance
     */
    @Override
    public void init() {
        Robot.init(hardwareMap, telemetry, gamepad1, gamepad2);
        isRecording = false;
        telemetry.addData("FilePath: ", Robot.filePath);
        try {
            recorder = new RobotRecorder(Robot.filePath, Robot.fileName);
        }
        catch(IOException e) {
            telemetry.addData("Init IO error", e);
            telemetry.update();
        }
    }


    /**
     * Runs normal robot teleop, along with the option to record the robot's movements
     */
    @Override
    public void loop() {
        Robot.run();
        //Click Dpad down to begin recording
        if (gamepad1.dpad_right)
            isRecording = true;

        //Click Dpad up to stop recording
        if (gamepad1.dpad_left) {
            if(isRecording)
                try {
                    recorder.stopRecording();
                }
                catch (IOException e) {
                    telemetry.addData("Stoping record IO error", e);
                }
            isRecording = false;
        }

        // Record if dpad down was clicked
        if (isRecording)
        {
            try {
                recorder.record();
                sleep(35); //This is needed so that the RecordReader can stay in tune with the recordings
            } catch (IOException e) {
                telemetry.addData("Recording IO error", e);

            }
        }

        telemetry.addData("Recording", isRecording);
        telemetry.update();

    }



    private void sleep(long milliseconds)
    {
        long sleepStart = System.currentTimeMillis();
        while( milliseconds > System.currentTimeMillis() - sleepStart)
        { }
    }


    /**
     * Stops the robot movements and the recorder if needed.
     */
    @Override
    public void stop() {
        Robot.stop();
        if(recorder != null && !recorder.isWriterClosed())
        {
            try {
                recorder.stopRecording();
            }
            catch (IOException e) {
                telemetry.addData("Stoping record IO error", e);
                telemetry.update();
            }
        }
    }
}
