package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous(name="Waffle Move END CLOSE", group="Build Side Autos")
public class MovePlatformEndClose extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap,telemetry, gamepad1, gamepad2, this);
        Robot.driveTrain.setBrakeMode(false);
        boolean isRed = Robot.colorPicker.isRed();
        waitForStart();


        new MovePlatformMovement().run(isRed);
        Robot.driveTrain.encoderDrive(0.5,45, 45, 3);
    }
}
