package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.PIDControllerConfigurable;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrainSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.vision.LimelightHelpers.RawFiducial;

public class FollowAprilTag extends Command {
  private final CommandSwerveDrivetrainSubsystem drivetrain;
  private final LimelightSubsystem limelight;

  private static final PIDControllerConfigurable rotationalPidController = new PIDControllerConfigurable(0.02 * Math.PI * 2.0, 0, 0, 2);
  private static final PIDControllerConfigurable xPidController = new PIDControllerConfigurable(0.02, 0, 0, 0.3);
  private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

  public FollowAprilTag(CommandSwerveDrivetrainSubsystem drivetrain, LimelightSubsystem limelight) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    addRequirements(this.drivetrain, this.limelight);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    RawFiducial fiducial = limelight.getFiducialWithId(5);

    drivetrain.applyRequest(() -> alignRequest.withRotationalRate(rotationalPidController.calculate(fiducial.txnc, 0)).withVelocityX(xPidController.calculate(fiducial.distToRobot, 2.25)));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequest(() -> idleRequest);
  }
}
