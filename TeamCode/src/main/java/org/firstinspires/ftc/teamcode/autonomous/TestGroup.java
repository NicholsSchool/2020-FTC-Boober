package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.util.commandstructure.CommandGroup;

public class TestGroup extends CommandGroup {

    public TestGroup()
    {
        addSequential(new DRElevator(2, true));
        addSequential(new DRIntake(1.0, 2));

    }
}
