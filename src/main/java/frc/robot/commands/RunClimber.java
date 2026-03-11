package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class RunClimber extends Command {
    /** Creates a new RunClimb. */
    private Climb m_climber;
            private boolean climberOn;
            
            public void runClimber(Climb climber, boolean on) {
                m_climber = climber;
            this.climberOn = on;
    }

    @Override
    public void initialize() {
        System.out.println("Run Climber Command Initialize");


    }
    @Override
    public void execute() {
           
       if (climberOn == true){

           
            m_climber.setPosition(2);
            //2 is an arbitrary value, change this as soon as possible to fit actual needs
           }

           if (climberOn == false) {
            m_climber.setPosition(0);
            // This is also subject to change
           }

           else {
            System.out.println("No value for RunClimber");
           }
        }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}

