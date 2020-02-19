package org.firstinspires.ftc.teamcode.roadrunner;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.util.RobotMap;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.getMotorVelocityF;

/*
 * Optimized tank drive implementation for REV ExHs. The time savings may significantly improve
 * trajectory following performance with moderate additional complexity.
 */

@Config
public class RoadRunnerTankDriveOptimized extends RoadRunnerTankDriveBase {
    private List<DcMotorEx> motors, leftMotors, rightMotors;
    private BNO055IMU imu;

    public RoadRunnerTankDriveOptimized(HardwareMap hardwareMap) {
        super();


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        BNO055IMUUtil.remapAxes(imu, AxesOrder.ZYX, AxesSigns.NPN);

        motors = Arrays.asList(RobotMap.lmDrive, RobotMap.lbDrive, RobotMap.rmDrive, RobotMap.rbDrive);
        leftMotors = Arrays.asList(RobotMap.lmDrive, RobotMap.lbDrive);
        rightMotors = Arrays.asList(RobotMap.rmDrive, RobotMap.rbDrive);

        for (DcMotorEx motor : motors) {
            if (RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }


        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftMotors.get(0).getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, getMotorVelocityF()
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        double leftSum = 0, rightSum = 0;
        RobotMap.runEncoderBulkRead();


        for (DcMotorEx leftMotor : leftMotors) {
            leftSum += encoderTicksToInches(leftMotor.getCurrentPosition());
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rightSum += encoderTicksToInches(rightMotor.getCurrentPosition());
        }
        return Arrays.asList(leftSum / leftMotors.size(), rightSum / rightMotors.size());
    }

    @Override
    public List<Double> getWheelVelocities() {
        double leftSum = 0, rightSum = 0;
        RobotMap.runEncoderBulkRead();


        for (DcMotorEx leftMotor : leftMotors) {
            leftSum += encoderTicksToInches(leftMotor.getVelocity());
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rightSum += encoderTicksToInches(rightMotor.getVelocity());
        }
        return Arrays.asList(leftSum / leftMotors.size(), rightSum / rightMotors.size());
    }

    @Override
    public void setMotorPowers(double v, double v1) {
        for (DcMotorEx leftMotor : leftMotors) {
            leftMotor.setPower(v);
        }

        for (DcMotorEx rightMotor : rightMotors) {
            rightMotor.setPower(v1);
        }
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public void stop()
    {
        for (DcMotorEx leftMotor : leftMotors) {
            leftMotor.setPower(0);
        }

        for (DcMotorEx rightMotor : rightMotors) {
            rightMotor.setPower(0);
        }
    }
}
