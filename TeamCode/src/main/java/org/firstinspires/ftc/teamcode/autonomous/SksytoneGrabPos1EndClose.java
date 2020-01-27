package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous(name="Skystone End Close", group="Loading Zone Autos")
public class SksytoneGrabPos1EndClose extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap,telemetry, gamepad1, gamepad2, this);
        SkystoneGrabPos1Movement movement = new SkystoneGrabPos1Movement();
        movement.setDistanceAwayFromStone(20);
        movement.runMovement();
    }
}
