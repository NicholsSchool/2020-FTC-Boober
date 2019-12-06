package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;

@Autonomous(name="AutoTests", group="Robot Auto")
public class AutoTests extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot.init(hardwareMap, FtcDashboard.getInstance().getTelemetry(), gamepad1, gamepad2, this);

        waitForStart();
//        RobotMap.telemetry.addLine("About to Drive");
//        RobotMap.telemetry.update();
//        Robot.driveTrain.encoderDrive(0.3, 12, 12);
//        RobotMap.telemetry.addLine("About to Turn");
//        RobotMap.telemetry.update();
//        Robot.driveTrain.PIDTurn(0.3, 90);
//        RobotMap.telemetry.update();

//        Robot.driveTrain.turn(0.4, 80);
        Robot.driveTrain.setBrakeMode(false);
        Robot.driveTrain.halfEncoderDrive(0.5, 33, 33);
        Robot.driveTrain.halfEncoderDrive(0.5, 33, 12);
    }


}
