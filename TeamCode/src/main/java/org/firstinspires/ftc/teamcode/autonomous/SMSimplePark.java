package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.Robot;

import java.util.ArrayList;

public class SMSimplePark extends OpMode {
    boolean isRed;
    final int DRIVE = 0, TURN = 1;
    final double  leftTurn = 80, rightTurn = -leftTurn;
    private Movement driveMovements, turnMovements;
    int state;
    private class Movement {
        private double[][] movements;
        private int index;

        public Movement(double[][] movements) {
            this.movements = movements;
            index = 0;
        }
        public Movement(double[] moves)
        {
            movements = new double[moves.length][1];
            for(int i = 0; i < moves.length; i ++)
                movements[i] = new double[]{moves[i]};
        }

        public double[] getNextMovement()
        {
            if(index >= movements.length)
                return new double[]{0, 0};
            return movements[index ++];
        }

    }

    @Override
    public void init() {
        Robot.init(hardwareMap, telemetry, gamepad1, gamepad2);
        Robot.driveTrain.setBrakeMode(false);
        isRed = Robot.colorPicker.isRed();
        driveMovements = new Movement(new double[][]{
                {12, 12}, {3, 3}
        });
        if(isRed)
            turnMovements = new Movement(new double[]{leftTurn});
        else
            turnMovements = new Movement(new double[]{rightTurn});


    }

    @Override
    public void loop() {

    }

    @Override
    public void stop(){

    }
}


