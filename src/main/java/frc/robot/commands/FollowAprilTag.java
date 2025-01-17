package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.TunerConstants;
import frc.robot.lib.PIDControllerConfigurable;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrainSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.vision.LimelightHelpers.RawFiducial;

public class FollowAprilTag extends Command {
  private final CommandSwerveDrivetrainSubsystem drivetrain;
  private final LimelightSubsystem limelight;

  private static final PIDControllerConfigurable rotationalPidController = new PIDControllerConfigurable(
      0.05, 0, 0, 0.5);
  private static final PIDControllerConfigurable xPidController = new PIDControllerConfigurable(0.3, 0, 0, 0.2);
  private static final PIDControllerConfigurable yPidController = new PIDControllerConfigurable(0.3, 0, 0, 0.5);
  private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

  // private static final SwerveRequest.SwerveDriveBrake brake = new
  // SwerveRequest.SwerveDriveBrake();

  public FollowAprilTag(CommandSwerveDrivetrainSubsystem drivetrain, LimelightSubsystem limelight) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    // addRequirements(this.drivetrain, this.limelight);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    RawFiducial fiducial;
    try {
      fiducial = limelight.getFiducialWithId(4);

      final double rotationalRate = rotationalPidController.calculate(fiducial.txnc, 0) * TunerConstants.MaxAngularRate
          * 0.2;
      final double velocityX = xPidController.calculate(fiducial.distToRobot, 2.25) * -1.0 * TunerConstants.MaxSpeed
          * 0.5;
      // final double velocityY = yPidController.calculate(fiducial.tync, 0) *
      // TunerConstants.MaxSpeed * 0.3;

      if (rotationalPidController.atSetpoint() && xPidController.atSetpoint() && yPidController.atSetpoint()) {
        this.end(true);
      }

      SmartDashboard.putNumber("txnc", fiducial.txnc);
      SmartDashboard.putNumber("distToRobot", fiducial.distToRobot);
      SmartDashboard.putNumber("rotationalPidController", rotationalRate);
      SmartDashboard.putNumber("xPidController", velocityX);
      drivetrain.setControl(
          alignRequest.withRotationalRate(rotationalRate).withVelocityX(velocityX));
      // .withVelocityY(velocityY));
      // drivetrain.applyRequest(() -> alignRequest.withRotationalRate(0.5 *
      // MaxAngularRate)
      // .withVelocityX(xPidController.calculate(0.2 * MaxSpeed)));
      // drivetrain.setControl(brake);
    } catch (LimelightSubsystem.NoSuchTargetException nste) {
    }
  }

  @Override
  public boolean isFinished() {
    // return rotationalPidController.atSetpoint() && xPidController.atSetpoint()
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequest(() -> idleRequest);
  }
}
