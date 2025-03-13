package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Constants;
import frc.robot.subsystems.Limelights.LimelightHelpers;
import frc.robot.generated.TunerConstants;
//import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore = false;
  private Timer dontSeeTagTimer, stopTimer;
  private CommandSwerveDrivetrain drivebase;
  private double tagID = -1;

  private double X_REEF_ALIGNMENT_P = 0.058;
  private double Y_REEF_ALIGNMENT_P = 0.33;
  private double ROT_REEF_ALIGNMENT_P = 0.058;

  private double X_SETPOINT_REEF_ALIGNMENT = 0;
  private double Y_SETPOINT_REEF_ALIGNMENT = 0.3;
  private double ROT_SETPOINT_REEF_ALIGNMENT = 0;

  private double X_TOLERANCE_REEF_ALIGNMENT = 0.01;
  private double Y_TOLERANCE_REEF_ALIGNMENT = 0.01;
  private double ROT_TOLERANCE_REEF_ALIGNMENT = 0;

  private double DONT_SEE_TAG_WAIT_TIME = 1;
  private double POSE_VALIDATION_TIME = 1;

private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors



  public AlignToReefTagRelative(boolean isRightScore, CommandSwerveDrivetrain drivebase) {
    xController = new PIDController(X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
    yController = new PIDController(Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? Y_SETPOINT_REEF_ALIGNMENT : -Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("limelight-three");

    System.out.println("command did run");
    
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight-three") && LimelightHelpers.getFiducialID("limelight-three") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-three");
      SmartDashboard.putNumber("x", postions[1]);
      SmartDashboard.putNumber("z", postions[2]);

      double xSpeed = xController.calculate(postions[0]);
      SmartDashboard.putNumber("xspeed", xSpeed);
      double ySpeed = -yController.calculate(postions[2]);
      double rotValue = -rotController.calculate(postions[4]);

      //drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);
      drivebase.setControl(
     drive.withVelocityX(Math.abs(xSpeed)) // Drive forward with negative Y (forward)
          .withVelocityY(0) // Drive left with negative X (left)
          .withRotationalRate(rotValue) // Drive counterclockwise with negative X (left)
     
      /* drive.withVelocityX(xSpeed * MaxSpeed) // Drive forward with negative Y (forward)
          .withVelocityY(ySpeed * MaxSpeed) // Drive left with negative X (left)
          .withRotationalRate(rotValue * MaxSpeed) // Drive counterclockwise with negative X (left)
 */ 
);

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      //drivebase.drive(new Translation2d(), 0, false);
      drivebase.setControl(
      drive.withVelocityX(0) // Drive forward with negative Y (forward)
          .withVelocityY(0) // Drive left with negative X (left)
          .withRotationalRate(0) // Drive counterclockwise with negative X (left)
      );
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
    //System.out.println(postions);
  }

  @Override
  public void end(boolean interrupted) {
    //drivebase.drive(new Translation2d(), 0, false);
    drivebase.applyRequest(() ->
      drive.withVelocityX(0) // Drive forward with negative Y (forward)
          .withVelocityY(0) // Drive left with negative X (left)
          .withRotationalRate(0) // Drive counterclockwise with negative X (left)
      );
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(POSE_VALIDATION_TIME);
  }      



}