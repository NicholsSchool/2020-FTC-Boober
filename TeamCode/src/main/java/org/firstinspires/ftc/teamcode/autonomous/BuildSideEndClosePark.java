package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.movements.ParkMovement;
import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous(name="Build Side Park Close", group="Build Side Autos")
public class BuildSideEndClosePark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap,telemetry, gamepad1, gamepad2, this);
        Robot.driveTrain.setBrakeMode(false);
        boolean isRed = Robot.colorPicker.isRed();
        waitForStart();
        new ParkMovement().runMovement(isRed,false, false );
    }
}
