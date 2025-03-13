// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlignToReefTagRelative;
import frc.robot.constants.TunerConstants;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.commands.AlignToReef;
import frc.robot.commands.FollowAprilTag;
import frc.robot.lib.LimelightConfig;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrainSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;


public class RobotContainer {
    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);
    
    // Subsystems
    private final CommandSwerveDrivetrainSubsystem drivetrain = TunerConstants.createDrivetrain();
    // private final LimelightSubsystem limelightThree = new LimelightSubsystem(
    // new LimelightConfig("limelight-seven", Inches.of(10.5).in(Meters),
    // Inches.of(9.375).in(Meters),
    // Inches.of(11.5).in(Meters), 0, 0, 0));
    
    private final LimelightSubsystem limelightFrontLeft = new LimelightSubsystem(
    new LimelightConfig("limelight-five", Inches.of(10.5).in(Meters), Inches.of(9.375).in(Meters),
    Inches.of(11.5).unaryMinus().in(Meters), 0, 0, 0));
    
    // private final LimelightSubsystem limelightFrontRight = new LimelightSubsystem(
    // new LimelightConfig("limelight-fifteen", Inches.of(14.5).in(Meters), 0,
    // Inches.of(8.25).in(Meters), 0, 0, 0));
    
    private final LimelightSubsystem limelightFrontRight = new LimelightSubsystem(
    new LimelightConfig("limelight-fifteen", Inches.of(10.5).in(Meters),
    Inches.of(9.375).in(Meters),
    Inches.of(11.5).in(Meters), 0, 0, 0));
    
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
        
        // reset the field-centric heading on left bumper press
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        driverController.leftTrigger().whileTrue(new AlignToReef(drivetrain, limelightFrontRight).withTimeout(10));
        driverController.rightTrigger().whileTrue(new AlignToReef(drivetrain, limelightFrontLeft).withTimeout(10));
        
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
        
        drivetrain.registerTelemetry(drivetrain.logger::telemeterize);
    }
    
    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
