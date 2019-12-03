package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.record.RobotRecorder;

import java.io.IOException;

@TeleOp(name="Teleop Record Test", group="Recording Tests")
public class Record extends OpMode {
    private RobotRecorder recorder;
    private boolean isRecording;

    @Override
    public void init() {
        Robot.init(hardwareMap, telemetry, gamepad1, gamepad2);
        /*
           !!!! MAY HAVE TO EDIT AndroidManifest.xml TO SAVE FILES !!!
           May have to insert:
           <uses-permission android:name="android.permission.WRITE_EXTERNAL_STORAGE"/>
         */
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
                sleep(35);
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
