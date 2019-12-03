package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.EncoderDrive;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;
import org.firstinspires.ftc.teamcode.util.commandstructure.Command;

@Autonomous(name="AutoTests", group="Robot Auto")
public class AutoTests extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot.init(hardwareMap, FtcDashboard.getInstance().getTelemetry(), gamepad1, gamepad2, this);
        Command autoCommand = new EncoderDrive(12, 12, -0.3);
        waitForStart();
        // Robot.driveTrain.encoderDrive(0.3, 12, 12);
        while (opModeIsActive()) {
            autoCommand.run();
            RobotMap.telemetry.update();
        }
    }


}
