package org.firstinspires.ftc.teamcode.sensors;

import org.firstinspires.ftc.teamcode.util.RobotMap;

public class ColorPicker {

    /**
     * Returns true if the color dial is set to red
     * @return true if the color dial is set to red
     */
    public boolean isRed()
    {
        double position = RobotMap.colorDial.getVoltage();
        return position < 1.9 && position > 0.5;
    }
}
