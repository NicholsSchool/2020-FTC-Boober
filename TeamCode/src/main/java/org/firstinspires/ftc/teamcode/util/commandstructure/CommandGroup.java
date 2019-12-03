package org.firstinspires.ftc.teamcode.util.commandstructure;

import java.util.ArrayList;

public class CommandGroup {

    ArrayList<CommandEntry> commands;
    ArrayList<CommandEntry> parallelGroup;
    private int command_index; // Using an index to keep track of commands rather than .remove(0) becuase in case
                                // I want to loop through the commands again for some reason

    private CommandEntry currentCommand;
    private boolean parallelGroupRunning;
    public CommandGroup()
    {
        commands = new ArrayList<>();
        parallelGroup = new ArrayList<>();
        command_index = 0;
    }

    public final void addSequential(Command command)
    {
        commands.add(new CommandEntry(command, CommandEntry.SEQUENTIAL));
    }

    public final void addParallel(Command command)
    {
        commands.add(new CommandEntry(command, CommandEntry.PARALLEL));
    }

    private boolean checkedAllCommands()
    {
        return  command_index >= commands.size();
    }

    public void run()
    {
        if(checkedAllCommands() )
            return;

        // Get a command from the list if currently don't have one to run
        if(currentCommand == null) {
            currentCommand = commands.get(command_index ++);
        }

        //If the parallel group is not running and the command is sequential run this until
        // the run() is true, then end it and remove command
        if(currentCommand.type == CommandEntry.SEQUENTIAL && !parallelGroupRunning) {
            if (currentCommand.command.run()) {
                currentCommand.command.end();
                currentCommand = null;
            }
        }

        //If the command is not sequential it is parallel, or the parallelGroup is running and needs to continue
        else
        {
            //If the group is empty and the code reaches this point, the current command has to be
            // parallel, so keep adding parallel commands in the list until you reach a sequential or end of list
            if(parallelGroup == null || parallelGroup.size() == 0)
                while(currentCommand.type == CommandEntry.PARALLEL && !checkedAllCommands())
                {
                    parallelGroup.add(currentCommand);
                    currentCommand = commands.get(command_index ++);
                }

            // Run through the group, if it is done, end and remove the command
            for(int i = 0; i < parallelGroup.size(); i ++)
            {
                if(parallelGroup.get(i).command.run()) {
                    parallelGroup.get(i).command.end();
                    parallelGroup.remove(i--); // minus is because the loop is an increasing loop
                }
            }
            // The parallel group should continue to run until all parallel commands finish
            parallelGroupRunning = parallelGroup.size() > 0;
        }
    }

    private static class CommandEntry{
        public static final int SEQUENTIAL = 0, PARALLEL = 1;
        private final int type;
        private final Command command;
        CommandEntry(Command c, int type)
        {
            this.type = type;
            this.command = c;
        }
    }
}
