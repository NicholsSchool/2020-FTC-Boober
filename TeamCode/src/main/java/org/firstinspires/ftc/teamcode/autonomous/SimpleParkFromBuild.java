package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous(name="Simple Park FROM BUILD SIDE", group="Build Side Autos")
public class SimpleParkFromBuild extends LinearOpMode {
    private double driveSpeed = 0.5, turnSpeed = 0.4,
            leftTurn = 80, rightTurn = -leftTurn;
    @Override
    public void runOpMode() {
        Robot.init(hardwareMap, telemetry, gamepad1, gamepad2, this);
        Robot.driveTrain.setBrakeMode(false);
        waitForStart();
        run(Robot.colorPicker.isRed());
    }

    public void run(boolean isRed) {
        Robot.driveTrain.encoderDrive(driveSpeed, 18, 18, 3);
        if(isRed)
            Robot.driveTrain.turn(turnSpeed, leftTurn, 3);
        else
            Robot.driveTrain.turn(turnSpeed, rightTurn, 3);
        Robot.driveTrain.encoderDrive(driveSpeed, 4, 4, 3);

    }
}
