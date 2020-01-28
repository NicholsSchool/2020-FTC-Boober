package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.movements.FaceBuildZone;
import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous(name="Simple Park", group="Loading Side Autos")
public class SimplePark extends LinearOpMode {

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
        new FaceBuildZone(driveSpeed, turnSpeed, leftTurn, rightTurn).run(isRed);
        Robot.driveTrain.encoderDrive(driveSpeed, 33, 33, 3);

    }
}
