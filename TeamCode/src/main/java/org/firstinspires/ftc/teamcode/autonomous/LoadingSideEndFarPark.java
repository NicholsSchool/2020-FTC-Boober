package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.movements.ParkMovement;
import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous(name="Loading Zone Park Far", group="Loading Zone Autos")
public class LoadingSideEndFarPark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap,telemetry, gamepad1, gamepad2, this);
        Robot.driveTrain.setBrakeMode(false);
        boolean isRed = Robot.colorPicker.isRed();
        waitForStart();
        new ParkMovement().runMovement(isRed,true, true );
    }
}
