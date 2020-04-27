package org.firstinspires.ftc.teamcode.util.record;

import android.os.Environment;

import org.firstinspires.ftc.teamcode.util.Robot;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class RobotRecorder {
    private FileWriter writer;
    private boolean startedWriting;
    private long start;
    private boolean writerClosed;

    public RobotRecorder(String fileName) throws IOException {
        writer = new FileWriter(new File(Environment.getExternalStorageDirectory().getPath(), fileName));
        start = System.currentTimeMillis();
        writerClosed = false;
        startedWriting = false;
    }

    private void initWriting()
    {
        start = System.currentTimeMillis();
        startedWriting = true;
    }

    public void record() throws IOException {
        if(writer == null)
            return;

        if(!startedWriting)
            initWriting();

        writer.append("" + (System.currentTimeMillis() - start));

        //Code for recording everything
        for (String s : Robot.getSubsystems().keySet()) {
            if(!(Robot.getSubsystems().get(s) instanceof  Recordable))
                continue;
            writer.append("," + s + "|");
            for (double d : ((Recordable) Robot.getSubsystems().get(s)).getValues())
                writer.append(d + "|");
        }
        writer.append("\n");
    }

    public void stopRecording() throws IOException {
        if(writer != null) {
            writer.flush();
            writer.close();
        }
        writerClosed = true;

    }

    public boolean isWriterClosed()
    {
        return writerClosed;
    }
}
