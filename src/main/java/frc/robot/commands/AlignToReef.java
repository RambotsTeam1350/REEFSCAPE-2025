package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.TunerConstantsPracticeBot;
import frc.robot.lib.PIDControllerConfigurable;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrainSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.vision.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.vision.LimelightHelpers.RawFiducial;

public class AlignToReef extends Command {
  private final CommandSwerveDrivetrainSubsystem drivetrain;
  private final LimelightSubsystem limelight;

  private static final PIDControllerConfigurable rotationalPidController = new PIDControllerConfigurable(
      1.5, 0.03, 0.1, 0.4)
      .withInputFilter(3)
      .withOutputFilter(2)
      .withDeadband(0.05)
      .withOutputRateLimit(0.1);
  private static final PIDControllerConfigurable xPidController = new PIDControllerConfigurable(
      0.55, 0, 0.05, 0.06)
      .withInputFilter(3)
      .withOutputFilter(2)
      .withDeadband(0.01)
      .withOutputRateLimit(0.1);
  private static final PIDControllerConfigurable yPidController = new PIDControllerConfigurable(
      0.55, 0, 0.05, 0.06)
      .withInputFilter(3)
      .withOutputFilter(2)
      .withDeadband(0.01)
      .withOutputRateLimit(0.1);
  private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

  // private static final SwerveRequest.SwerveDriveBrake brake = new
  // SwerveRequest.SwerveDriveBrake();

  public AlignToReef(CommandSwerveDrivetrainSubsystem drivetrain, LimelightSubsystem limelight) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    // addRequirements(this.drivetrain, this.limelight);
  }

  @Override
  public void initialize() {
    System.out.println("COMMAND RAN");
  }

  @Override
  public void execute() {
    LimelightTarget_Fiducial fiducial = null;
    try {
      fiducial = limelight.getFiducial();

      Pose3d targetPoseInRobotSpace = fiducial.getTargetPose_CameraSpace();
      double distToRobot = targetPoseInRobotSpace.getZ();
      double sideError = targetPoseInRobotSpace.getX();
      SmartDashboard.putNumber("getRotation", targetPoseInRobotSpace.getRotation().getY());
      double rotationalError = targetPoseInRobotSpace.getRotation().getY();

      double rotationalRate = rotationalPidController.calculate(rotationalError, 0)
          * TunerConstantsPracticeBot.MaxAngularRate
          * 0.6;
      final double velocityX = xPidController.calculate(distToRobot, Inches.of(10).in(Meters)) * -1.0
          * TunerConstantsPracticeBot.MaxSpeed
          * 0.6;
      final double velocityY = yPidController.calculate(sideError, 0) * 1.0 *
          TunerConstantsPracticeBot.MaxSpeed * 0.6;

      // if (!xPidController.atSetpoint() || !yPidController.atSetpoint()) {
      //   rotationalRate /= 5;
      // }

      // if (rotationalPidController.atSetpoint() && xPidController.atSetpoint() &&
      // yPidController.atSetpoint()) {
      // this.end(true);
      // }

      SmartDashboard.putNumber("txnc", fiducial.tx_nocrosshair);
      SmartDashboard.putNumber("distToRobot", distToRobot);
      SmartDashboard.putNumber("rotationalPidController", rotationalRate);
      SmartDashboard.putNumber("xPidController", velocityX);
      drivetrain.setControl(
          alignRequest.withVelocityX(velocityX)
              .withVelocityY(velocityY));
      drivetrain.setControl(alignRequest.withRotationalRate(rotationalRate));
    } catch (LimelightSubsystem.NoSuchTargetException nste) {
      SmartDashboard.putNumber("TEST", -1);
    }
  }

  @Override
  public boolean isFinished() {
    return rotationalPidController.atSetpoint() && xPidController.atSetpoint() && yPidController.atSetpoint();
    // return false;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequest(() -> idleRequest);
  }
}

