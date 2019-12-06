package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.movements.FaceBuildZone;
import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous(name="Move Platform and Park", group="Autos")
public class MovePlatform extends LinearOpMode {
    private double driveSpeed = 0.5, turnSpeed = 0.3,
            leftTurn = 80, rightTurn = -leftTurn;
    @Override
    public void runOpMode() {
        Robot.init(hardwareMap, FtcDashboard.getInstance().getTelemetry(), gamepad1, gamepad2, this);
        waitForStart();
        run(Robot.colorPicker.isRed());
    }

    public void run(boolean isRed) {
        new FaceBuildZone(driveSpeed, turnSpeed, leftTurn, rightTurn).run(isRed);
        Robot.driveTrain.halfEncoderDrive(driveSpeed, 85, 85);
        if(isRed)
            Robot.driveTrain.turn(turnSpeed, rightTurn);
        else
            Robot.driveTrain.turn(turnSpeed, leftTurn);

    }
}
