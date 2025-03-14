package frc.robot.subsystems.intakeCoral;

import javax.security.auth.callback.ConfirmationCallback;

import frc.robot.susbystem.CoralModuleSubsystem;

public class intakeCoral {
    private final CoralModuleSubsystem coralModuleSubsystem = new CoralModuleSubsystem();

    public Command intakeCoral(){
        return Commands.sequence (
                Commands.runOnce(()  -> coralModuleSubsystem.IntakeCoral())
        );
    }
}
