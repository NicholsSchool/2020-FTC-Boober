package org.firstinspires.ftc.teamcode.sensors;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.util.RobotMap;
import org.firstinspires.ftc.teamcode.sensors.SkystoneDetector.RobotPosition;


public class Camera {
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AVhfqRH/////AAABGd7wUb568kaDho9qW6uyIZV9ovIZF9UnMCqNzmBE1YeaiqGsmXkyZr3aGikHN++7DfnOeymbsUDQELp8AGTQRbYXf6re9h7csCPKXnY/YjlbOHCp7hzDRIJ3rXe+m1RmOIDjLUs8w6sauRzlhGH6qlWfqvBrp94N2NUMygqt4MMDlrXH5B2ieMgcaUJiA3yS9U27wLKcXzPLzhNa2Pj6uyDXAMIYC2ymfRVVOOecwr9wImJ5fiHjzXvJTwPoQ9hYgEn92jPl2Z+yEq225/hdTGSgTKhlFRQI5sM4otsL/vCH6avqjuwyTBC5189St7ZrMzNjsBIRTsTLOqlwFdasebS4d9hHGxfioYlq4+4fSvSF";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;
    private SkystoneDetector skystoneDetector;

    public Camera()
    {
        initVuforia();

        skystoneDetector = new SkystoneDetector(vuforia);

    }

    public int getSkystonePosition(boolean saveBitmaps, boolean isRed, int robotPosition)
    {
        RobotPosition robotPositionType = RobotPosition.BLUE_POSITION1;
        if(isRed)
        {
            //  robotPositionType = RobotPosition.RED_POSITION1;
            // if(robotPosition == 2)
//                robotPosition = RobotPosition.RED_POSITION2;
        }
        else
        {
            robotPositionType = RobotPosition.BLUE_POSITION1;
//            if(robotPosition == 2)
//                robotPosition = RobotPosition.BLUE_POSITION2;
        }
        SkystoneDetector.SkystonePosition position =  skystoneDetector.getSkystonePosition(saveBitmaps, isRed, robotPositionType);
        RobotMap.telemetry.addData("Position ", position);
        switch(position)
        {
            case FAR:
                return 3;
            case CENTER:
                return 2;
            case NEAR:
                return 1;
            default:
                return 1;
        }
    }

    public int getSkystonePosition(boolean isRed, int robotPosition)
    {

        return getSkystonePosition(false, isRed, robotPosition);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        FtcDashboard.getInstance().startCameraStream(vuforia, 0);
    }

}
