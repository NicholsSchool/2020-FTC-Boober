package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.SkystoneGrabPos1Movement;
import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous(name="Skystone End Far", group="Loading Zone Autos")
public class SkystoneGrabPos1EndFar extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap,telemetry, gamepad1, gamepad2, this);
        SkystoneGrabPos1Movement movement = new SkystoneGrabPos1Movement();
        movement.runMovement();

    }
}
