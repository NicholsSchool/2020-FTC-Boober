package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.movements.SkystoneGrabMovement;
import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous(name="Skystone End Close", group="Loading Zone Autos")
public class SksytoneGrabEndClose extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap,telemetry, gamepad1, gamepad2, this);
        SkystoneGrabMovement movement = new SkystoneGrabMovement();
        movement.setDistanceAwayFromStone(23);
        movement.runMovement();
    }
}
