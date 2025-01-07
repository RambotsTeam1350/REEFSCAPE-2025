package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrainSubsystem;

public class FollowAprilTag extends Command {
  private final CommandSwerveDrivetrainSubsystem drivetrain;

  public FollowAprilTag(CommandSwerveDrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {

  }
}
