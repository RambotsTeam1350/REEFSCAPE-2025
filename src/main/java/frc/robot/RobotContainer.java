// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlignToReef;
import frc.robot.commands.FollowAprilTag;
import frc.robot.constants.TunerConstants;
import frc.robot.lib.LimelightConfig;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralModuleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrainSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;

public class RobotContainer {
  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController scoringController = new CommandXboxController(1);

  // Subsystems
  private final CommandSwerveDrivetrainSubsystem drivetrain = TunerConstants.createDrivetrain();
  // private final LimelightSubsystem limelightThree = new LimelightSubsystem(
  // new LimelightConfig("limelight-seven", Inches.of(10.5).in(Meters),
  // Inches.of(9.375).in(Meters),
  // Inches.of(11.5).in(Meters), 0, 0, 0));

  private final Pose3d frontRightLimelightPose = new Pose3d(Inches.of(10.5), Inches.of(9.375), Inches.of(11.5),
      new Rotation3d(0, 0, 0));
  private final Pose3d frontLeftLimelightPose = new Pose3d(Inches.of(10.5), Inches.of(9.375),
      Inches.of(11.5).unaryMinus(),
      new Rotation3d(0, 0, 0));

  private final LimelightSubsystem limelightFrontLeft = new LimelightSubsystem(
      new LimelightConfig("limelight-five", frontLeftLimelightPose));

  // private final LimelightSubsystem limelightFrontRight = new LimelightSubsystem(
  // new LimelightConfig("limelight-fifteen", Inches.of(14.5).in(Meters), 0,
  // Inches.of(8.25).in(Meters), 0, 0, 0));

  private final LimelightSubsystem limelightFrontRight = new LimelightSubsystem(
      new LimelightConfig("limelight-fifteen", frontRightLimelightPose));

  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final CoralModuleSubsystem coralModuleSubsystem = new CoralModuleSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ShoulderSubsystem shoulderSubsystem = new ShoulderSubsystem();
  private final WristSubsystem wristSubsystem = new WristSubsystem();

  public RobotContainer() {
    configureDrivetrainBindings();
  }

  private void configureDrivetrainBindings() {
    /* Setting up bindings for necessary control of the swerve drive platform */

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain
            .applyRequest(() -> drivetrain.driveRequest
                .withVelocityX(-driverController.getLeftY()
                    * TunerConstants.MaxSpeed) // Drive
                // forward
                // with
                // negative Y
                // (forward)
                .withVelocityY(-driverController.getLeftX()
                    * TunerConstants.MaxSpeed) // Drive left
                                               // with
                                               // negative
                                               // X (left)
                .withRotationalRate(-driverController.getRightX()
                    * TunerConstants.MaxAngularRate) // Drive
                                                     // counterclockwise
            // with
            // negative X (left)
            ));

    //////////////////////////////////////////////////////
    /// Driver controls
    //////////////////////////////////////////////////////
    driverController.a().whileTrue(drivetrain.applyRequest(() -> drivetrain.brakeRequest));
    driverController.b().whileTrue(drivetrain.applyRequest(
        () -> drivetrain.pointRequest.withModuleDirection(
            new Rotation2d(-driverController.getLeftY(),
                -driverController.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    driverController.back().and(driverController.y())
        .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driverController.back().and(driverController.x())
        .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driverController.start().and(driverController.y())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverController.start().and(driverController.x())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on start press
    driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    driverController.leftTrigger().whileTrue(new AlignToReef(drivetrain, limelightFrontRight));
    driverController.rightTrigger().whileTrue(new AlignToReef(drivetrain, limelightFrontLeft));

    driverController.povUp().whileTrue(
        drivetrain.applyRequest(() -> drivetrain.driveRequest
            .withVelocityX(TunerConstants.MaxSpeed * 0.25)));
    driverController.povDown().whileTrue(
        drivetrain.applyRequest(
            () -> drivetrain.driveRequest
                .withVelocityX(TunerConstants.MaxSpeed * 0.25 * -1.0)));
    driverController.povLeft().whileTrue(
        drivetrain.applyRequest(
            () -> drivetrain.driveRequest
                .withVelocityY(TunerConstants.MaxSpeed * 0.25)));
    driverController.povRight().whileTrue(
        drivetrain.applyRequest(
            () -> drivetrain.driveRequest
                .withVelocityY(TunerConstants.MaxSpeed * 0.25 * -1.0)));

    //////////////////////////////////////////////////////
    /// Scoring controls
    //////////////////////////////////////////////////////

    // scoringController.a().onTrue(elevatorSusbsystem.L1Command().alongWith(shoulderSubsystem.ShoulderToLevel1()).alongWith(wristSubsystem.WristToLevel1()));
    // scoringController.x().onTrue(elevatorSusbsystem.L2Command().alongWith(shoulderSubsystem.ShoulderToLevel2()).alongWith(wristSubsystem.WristToLevel2()));
    // scoringController.b().onTrue(elevatorSusbsystem.L3Command().alongWith(shoulderSubsystem.ShoulderToLevel3()).alongWith(wristSubsystem.WristToLevel3()));
    // scoringController.y().onTrue(elevatorSusbsystem.L4Command().alongWith(shoulderSubsystem.ShoulderToLevel4()).alongWith(wristSubsystem.WristToLevel4()));

    // scoringController.leftTrigger().onTrue(elevatorSusbsystem.LBargeCommand().alongWith(shoulderSubsystem.ShoulderToLevelBarge()).alongWith(wristSubsystem.WristToLevelBarge()));
    // scoringController.leftTrigger().onTrue(coralModuleSubsystem.deliverCoral()));

    //////////////////////////////////////////////////////

    drivetrain.registerTelemetry(drivetrain.logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
