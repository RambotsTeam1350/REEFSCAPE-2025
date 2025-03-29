// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.nio.file.Path;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AlignToReef;
import frc.robot.commands.FollowAprilTag;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.TunerConstantsPracticeBot;
import frc.robot.lib.LimelightConfig;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralModuleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.DeAlgaefierSubsystem;
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

  public final LimelightSubsystem limelightFrontLeft = new LimelightSubsystem(
      new LimelightConfig("limelight-three", frontLeftLimelightPose),
      false);

  private final LimelightSubsystem limelightFrontRight = new LimelightSubsystem(
    new LimelightConfig("limelight-fifteen", frontRightLimelightPose),
    false);
  
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final CoralModuleSubsystem coralModuleSubsystem = new CoralModuleSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final FunnelSubsystem funnelSubsystem = new FunnelSubsystem();
  private final DeAlgaefierSubsystem algaeSubsystem = new DeAlgaefierSubsystem();

  private final SendableChooser<Command> autoChooser;   

  public RobotContainer() {
    configureDrivetrainBindings();

    autoChooser = AutoBuilder.buildAutoChooser("blue middle");
    SmartDashboard.putData("Auto Chooser", autoChooser);
  

  NamedCommands.registerCommand("l2Command", elevatorSubsystem.l2Command());
  NamedCommands.registerCommand("l3Command", elevatorSubsystem.l3Command());
  NamedCommands.registerCommand("deliverCoral", coralModuleSubsystem.deliverCoral());
  NamedCommands.registerCommand("restPositionCommand", elevatorSubsystem.restPositionCommand());
  NamedCommands.registerCommand("IntakeCoralCommand", coralModuleSubsystem.IntakeCoralCommand());

}
  public void periodic() {

    System.out.println(autoChooser.getSelected());
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
   // driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    driverController.leftBumper().whileTrue(new AlignToReef(drivetrain, limelightFrontRight, -2));
    driverController.rightBumper().whileTrue(new AlignToReef(drivetrain, limelightFrontLeft, 2));
    

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

    scoringController.leftBumper().onTrue(coralModuleSubsystem.IntakeCoralCommand());
    scoringController.rightBumper().onTrue(coralModuleSubsystem.deliverCoral());
    scoringController.leftTrigger().onTrue(algaeSubsystem.deAlgaefierDown());
    scoringController.rightTrigger().onTrue(algaeSubsystem.deAlgaefierUp());

    scoringController.povDown().onTrue(climberSubsystem.ascendCommand());
    scoringController.povUp().onTrue(climberSubsystem.descendCommand());
    scoringController.povLeft().onTrue(funnelSubsystem.funnelOpen());
    scoringController.povRight().onTrue(funnelSubsystem.funnelClose());

    scoringController.a().onTrue(elevatorSubsystem.l1Command());
    scoringController.x().onTrue(elevatorSubsystem.l2Command());
    scoringController.b().onTrue(elevatorSubsystem.l3Command());
    scoringController.y().onTrue(elevatorSubsystem.l4Command());

    

    //////////////////////////////////////////////////////

    drivetrain.registerTelemetry(drivetrain.logger::telemeterize);
  }

   public Command getAutonomousCommand() {
    return this.autoChooser.getSelected();
    
  }
  
    /*public Command getAutonomousCommand() {
      try{
          // Load the path you want to follow using its name in the GUI
          PathPlannerPath path = PathPlannerPath.fromPathFile("red middle");
  
          // Create a path following command using AutoBuilder. This will also trigger event markers.
          return AutoBuilder.followPath(path);
      } catch (Exception e) {
          DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
          return Commands.none();
      }
    }
  // ...*/

 /*  public Command getAutonomousCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    return new PathPlannerAuto( "");
  }*/
}
