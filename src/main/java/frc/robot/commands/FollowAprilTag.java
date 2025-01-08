package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrainSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;

public class FollowAprilTag extends Command {
  private final CommandSwerveDrivetrainSubsystem drivetrain;
  private final LimelightSubsystem limelight;

  private static final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public FollowAprilTag(CommandSwerveDrivetrainSubsystem drivetrain, LimelightSubsystem limelight) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
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
