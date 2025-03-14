package frc.robot.subsystems.rightCoral;

import frc.robot.commands.FollowAprilTag;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AlignToReef;
import frc.robot.commands.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.CoralModuleSubsystem;


public class rightCoral {
    private final FollowAprilTag followAprilTag = new FollowAprilTag();

    public Command rightCoral(){
        return Commands.sequence(
            Commands.runOnce(()  -> AlignToReef(drivetrain, limelightFrontLeft)),
            Commands.runOnce(()  -> coralModuleSubsystem.deliverCoral())
        );
    }
}
