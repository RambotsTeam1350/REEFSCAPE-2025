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
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.Limelights.*;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    private final ElevatorSubsystem elevatorSusbsystem = new ElevatorSubsystem();
    private final ShoulderSubsystem shoulderSubsystem = new ShoulderSubsystem();
    private final WristSubsystem wristSubsystem = new WristSubsystem();
    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    private final LimelightSubsystemV2 limelightSubsystemV2 = new LimelightSubsystemV2();

    
    

    private final CommandXboxController joystick1 = new CommandXboxController(0); // Drive Base controller
    private final CommandXboxController joystick2 = new CommandXboxController(1); // Upper Mech conroller

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    private final AlignToReefTagRelative AlignToReefTagRelative = new AlignToReefTagRelative(false, drivetrain);
    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick1.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick1.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick1.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //joystick.b().whileTrue(drivetrain.applyRequest(() ->
            // point.withModuleDirection(new Rotation2d(-joystick1.getLeftY(), -joystick1.getLeftX()));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick1.back().and(joystick1.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick1.back().and(joystick1.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick1.start().and(joystick1.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick1.start().and(joystick1.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);
   
        ////////////////////////////////////////////////////////////////////////////////////
        /// Driver controls
        ////////////////////////////////////////////////////////////////////////////////////

        // reset the field-centric heading on start button press
        joystick1.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //joystick1.a().onTrue(elevatorSusbsystem.L1Command().alongWith(shoulderSubsystem.ShoulderToLevel1()).alongWith(wristSubsystem.WristToLevel1()));
        //joystick1.x().onFalse(elevatorSusbsystem.L2Command().alongWith(shoulderSubsystem.ShoulderToLevel2()).alongWith(wristSubsystem.WristToLevel2()));
        //joystick1.b().onTrue(elevatorSusbsystem.L3Command().alongWith(shoulderSubsystem.ShoulderToLevel3()).alongWith(wristSubsystem.WristToLevel3()));
        //joystick1.y().onFalse(elevatorSusbsystem.L4Command().alongWith(shoulderSubsystem.ShoulderToLevel4()).alongWith(wristSubsystem.WristToLevel4()));

        //joystick1.leftTrigger().onTrue(limelightSubsystem.alignToCoralReef("left"));
        joystick1.leftBumper().onTrue(AlignToReefTagRelative);
        joystick1.leftTrigger().onTrue(limelightSubsystemV2.alignToCoralReef("left"));
        joystick1.rightTrigger().onTrue(limelightSubsystem.alignToCoralReef("right"));

        ////////////////////////////////////////////////////////////////////////////////////
        /// Scoring controls
        ////////////////////////////////////////////////////////////////////////////////////
        joystick2.povDown().onTrue(climberSubsystem.ascendCommand());
        joystick1.povUp().onTrue(climberSubsystem.descendCommand());

        joystick2.a().onTrue(elevatorSusbsystem.L1Command().alongWith(shoulderSubsystem.ShoulderToLevel1()).alongWith(wristSubsystem.WristToLevel1()));
        joystick2.x().onTrue(elevatorSusbsystem.L2Command().alongWith(shoulderSubsystem.ShoulderToLevel2()).alongWith(wristSubsystem.WristToLevel2()));
        joystick2.b().onTrue(elevatorSusbsystem.L3Command().alongWith(shoulderSubsystem.ShoulderToLevel3()).alongWith(wristSubsystem.WristToLevel3()));
        joystick2.y().onTrue(elevatorSusbsystem.L4Command().alongWith(shoulderSubsystem.ShoulderToLevel4()).alongWith(wristSubsystem.WristToLevel4()));

        //joystick2.leftBumper().onTrue(shoulderSubsystem.ShoulderToLevel2());
        joystick2.leftTrigger().onTrue(elevatorSusbsystem.LBargeCommand().alongWith(shoulderSubsystem.ShoulderToLevelBarge()).alongWith(wristSubsystem.WristToLevelBarge()));
        // joystick2.leftTrigger().onTrue(coralModuleSubsystem.deliverCoral()));
    }


    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
