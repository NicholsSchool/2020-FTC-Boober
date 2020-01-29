package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.movements.MovePlatformMovement;
import org.firstinspires.ftc.teamcode.autonomous.movements.SkystoneGrabMovement;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;

@Autonomous(name="Skystone + Platform", group="Loading Zone Autos")
public class SkystoneGrabPlusMovePlatform extends LinearOpMode {
    private double driveSpeed = 1, turnSpeed = 0.4,
            leftTurn = 90, rightTurn = -leftTurn;
    private double driveTimeOut = 3, turnTimeOut = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.init(hardwareMap,telemetry, gamepad1, gamepad2, this);

        RobotMap.telemetry.addLine("Robot Init complete");
        RobotMap.telemetry.update();

        SkystoneGrabMovement stoneMovement = new SkystoneGrabMovement();
        stoneMovement.setDistanceAwayFromStone(23);
        stoneMovement.setGoBackDistance(115);
        stoneMovement.setPark(false);
        stoneMovement.setPullSpeed(0.4);
        stoneMovement.setDoubleCheck(false);
        RobotMap.telemetry.addLine("Stone Movement setupComplete");
        RobotMap.telemetry.update();


        MovePlatformMovement platformMovement = new MovePlatformMovement();
        platformMovement.setDistanceBackupToPlatform(25);
        RobotMap.telemetry.addLine("Platform Movement setup complete");
        RobotMap.telemetry.update();

        RobotMap.telemetry.addLine("All Setup complete, waiting for camera");
        RobotMap.telemetry.update();
        stoneMovement.runMovement();

        Robot.claw.timedMove(true, 1);
        platformMovement.grabPlatform(Robot.colorPicker.isRed());

        Robot.driveTrain.encoderDrive(0.5,37, 37, 3);
    }


}
