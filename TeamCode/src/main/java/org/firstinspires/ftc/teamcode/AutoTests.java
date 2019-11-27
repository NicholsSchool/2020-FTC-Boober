package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous(name="AutoTests", group="Robot Auto")
public class AutoTests extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot.init(hardwareMap, FtcDashboard.getInstance().getTelemetry(), gamepad1, gamepad2, this);

        waitForStart();
       // Robot.driveTrain.encoderDrive(0.3, 12, 12);
            Robot.driveTrain.colorDrive(-0.2);
    }


}
