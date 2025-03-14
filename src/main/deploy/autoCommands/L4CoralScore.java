package frc.robot.subsystems.L4CoralScore;

import frc.robot.subsystems.CoralModuleSubsystem;

public class L4CoralScore {
    
    private final CoralModuleSubsystem coralModuleSubsystem = new CoralModuleSubsystem();
    
    public Command L4CoralScore(){

        return Commands.sequence(
            Commands.runOnce(()  -> coralModuleSubsystem.deliverCoral())

        );
    }
}
