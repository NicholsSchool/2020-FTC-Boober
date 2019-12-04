package org.firstinspires.ftc.teamcode.util.commandstructure;

import org.firstinspires.ftc.teamcode.util.Robot;

public abstract class Command {

    /*
     Ideas, for parallel commands: make it possible so that run()
     can be changed to have both isFinished() in the while loop condition
     and then call both execute. Obviously will have to make it so that each finish when they have too.
     If I do this, I could/should probably make the requires() method, and make sure parallel commands
     don't require the same subsystems

     Notes:
     Have the run method initialize at start and then run execute and have the final line be returning isFinished(),
     This way, the command isn't responsible for looping itself, but a commandRunner class would, and could handle
     parallel stuff.

      FRC keeps an array of the parrallel commands to run, if its done, it removes that command
           if (!child.run()) {
        child.removed();
        m_children.removeElementAt(i--);
      }
     */

    public String name = "";

    private boolean isInitialized = false;

    public abstract void initialize();

    public abstract void execute();

    public abstract boolean isFinished();

    public abstract void end();

    public boolean run(){
        if(!isInitialized) {
            initialize();
            isInitialized = true;
        }
            execute();
        return isFinished();
    }

}
