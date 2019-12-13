package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;

@Autonomous(name = "loadingPark", group = "Loading Side Autos")

public class LoadingPark extends LinearOpMode {

    private double driveSpeed = 0.5, turnSpeed = 0.4,
            leftTurn = 80, rightTurn = -leftTurn;

    public void runOpMode() {
        Robot.init(hardwareMap, telemetry, gamepad1, gamepad2, this);
        Robot.driveTrain.setBrakeMode(false);
        waitForStart();
        run(Robot.colorPicker.isRed());


    }

    public void run(boolean isRed) {
        Robot.driveTrain.encoderDrive(driveSpeed, 27, 27, 3);
        if (isRed) {
            Robot.driveTrain.turn(turnSpeed, rightTurn);
        } else {
            Robot.driveTrain.turn(turnSpeed, leftTurn);
        }
        Robot.driveTrain.encoderDrive(driveSpeed, 8, 8, 3);


    }
}
