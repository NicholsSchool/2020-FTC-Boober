package org.firstinspires.ftc.teamcode.util.record;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

public class RecordReader {
    private Scanner reader;
    private long start, nextDataTime;
    private boolean reading;
    private boolean inTuneWithData;
    private boolean startedReading;

    public RecordReader(String filePath, String fileName) throws FileNotFoundException {
        reader = new Scanner(new File(filePath, fileName));

        reader.useDelimiter(",|\\n");
        start = System.currentTimeMillis();
        reading = true;
        inTuneWithData = true;

    }


    public boolean isReading()
    {
        return reading;
    }

    private void initReading()
    {
        startedReading = true;
        start = System.currentTimeMillis();
    }

    //for Testing
    long avgDiffInTime = 0;
    long count = 0;

    public void read() {
        if(reader == null)
            return;

        if(!startedReading)
            initReading();

        if (reader.hasNext()) {
            if (inTuneWithData)
                nextDataTime = reader.nextLong();

            long currentTime = System.currentTimeMillis() - start;
            long diffInTime = nextDataTime - currentTime;
            avgDiffInTime += diffInTime;
            count ++;
            RobotMap.telemetry.addData("Average Diff in Time", avgDiffInTime/count);
            RobotMap.telemetry.addData("Diff In Time", diffInTime );
            if (diffInTime <= 0) {
                RobotMap.telemetry.addData("Time", nextDataTime );
                RobotMap.telemetry.addData(" Current Time" , currentTime);
                while (reader.hasNext() && !reader.hasNextLong()) {
                    String[] values = reader.next().split("\\|");
                    String subsystemName = values[0];
                    double[] subsystemValues = new double[values.length - 1];
                    for(int i = 1; i < values.length; i ++)
                        subsystemValues[i - 1] = Double.parseDouble(values[i]);

                    ((Recordable) Robot.getSubsystems().get(subsystemName)).setValues(subsystemValues);

                }
                inTuneWithData = true;
            } else
                inTuneWithData = false;
        } else
            reading = false;
        RobotMap.telemetry.update();
    }

    public void stop()
    {
        if(reader != null)
            reader.close();
    }
}
