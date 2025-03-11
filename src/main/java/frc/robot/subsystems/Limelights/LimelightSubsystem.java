package frc.robot.subsystems.Limelights;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class LimelightSubsystem extends SubsystemBase {
  
  //private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight-one");
  private final NetworkTable limelightTable3;  
  private final NetworkTable limelightTable5;  
  
  private final NetworkTable limelightTable1;
  
  private final String limelightName5 = "limelight-five";
  private final String limelightName3 = "limelight-three";
  
  private final CommandSwerveDrivetrain drivetrain;
  
  private final double LIMELIGHT_HEIGHT = 12.96875; //height of the limelight camera off the ground, make sure to measure this as often as possible
  private final double TARGET_MOUNT_HEIGHT = 12.125; //height of the apriltag off the ground, in this case the one on the coral station
  private final double LIMELIGHT_MOUNT_ANGLE = 0.0; 
  private final double Ta = LimelightHelpers.getTA("limelight-five");
  
  private final int[] CORAL_TAGS_RED = {6, 7, 8, 9, 10, 11};
  private final int[] CORAL_TAGS_BLUE = {17, 18, 19, 20, 21, 22};
  
  public final int[] getCoralTags(Alliance alliance) {
    if (alliance == Alliance.Red) {
      return CORAL_TAGS_RED;
    } else {
      return CORAL_TAGS_BLUE;
    }
  }
  
  public double[] getPoseData5() {
    Pose3d pose = LimelightHelpers.getBotPose3d_TargetSpace(limelightName5);
    System.out.println(pose);
    return LimelightHelpers.pose3dToArray(pose);
  }
  
  public double[] getPoseData3() {
    Pose3d pose = LimelightHelpers.getBotPose3d_TargetSpace(limelightName3);
    System.out.println(pose);
    return LimelightHelpers.pose3dToArray(pose);
  }
  
  @Override
  public void periodic() {
    double[] poseData5 = getPoseData5();
    
    if (poseData5 != null && poseData5.length >= 6) {
      SmartDashboard.putNumber("Pose X 3", poseData5[0]);
      SmartDashboard.putNumber("Pose Y 3", poseData5[1]);
      SmartDashboard.putNumber("Pose Z 3", poseData5[2]);
      SmartDashboard.putNumber("Roll 3", poseData5[3]);
      SmartDashboard.putNumber("Pitch 3", poseData5[4]);
      SmartDashboard.putNumber("Yaw 3", poseData5[5]);
    }
    
    double[] poseData3 = getPoseData3();
    if (poseData3 != null && poseData3.length >= 6) {
      SmartDashboard.putNumber("Pose X", poseData3[0]);
      SmartDashboard.putNumber("Pose Y", poseData3[1]);
      SmartDashboard.putNumber("Pose Z", poseData3[2]);
      SmartDashboard.putNumber("Roll", poseData3[3]);
      SmartDashboard.putNumber("Pitch", poseData3[4]);
      SmartDashboard.putNumber("Yaw", poseData3[5]);
    }
    
    // Update odometry with vision measurements from both Limelights
    updateOdometryWithVision(limelightName3);
    updateOdometryWithVision(limelightName5);
  }
  
  /**
  * Updates the drivetrain's odometry with vision measurements from the specified Limelight.
  * This helps improve the robot's position estimate and makes alignment more accurate.
  * 
  * @param limelightName The name of the Limelight to get pose data from
  */
  private void updateOdometryWithVision(String limelightName) {
    // Only update if we have a valid target
    if (LimelightHelpers.getTV(limelightName)) {
      // Get the robot's pose in field space from the Limelight
      double[] botpose = LimelightHelpers.getBotPose_wpiBlue(limelightName);
      
      // Check if we have valid pose data
      if (botpose != null && botpose.length >= 6) {
        // Create a Pose2d from the botpose data (x, y, rotation)
        edu.wpi.first.math.geometry.Pose2d visionPose = new edu.wpi.first.math.geometry.Pose2d(
        botpose[0], 
        botpose[1], 
        edu.wpi.first.math.geometry.Rotation2d.fromDegrees(botpose[5])
        );
        
        // Get the timestamp of when the image was captured
        double timestamp = LimelightHelpers.getLatency_Capture(limelightName) / 1000.0 + 
        LimelightHelpers.getLatency_Pipeline(limelightName) / 1000.0;
        
        // Add the vision measurement to the drivetrain's odometry
        drivetrain.addVisionMeasurement(visionPose, timestamp);
        
        // Log that we updated odometry
        SmartDashboard.putString("Vision Update " + limelightName, "Updated at " + timestamp);
      }
    }
  }
  
  public LimelightSubsystem() {
    
    limelightTable3 = NetworkTableInstance.getDefault().getTable("limelight-three");
    limelightTable5 = NetworkTableInstance.getDefault().getTable("limelight-five");
    
    limelightTable1 = NetworkTableInstance.getDefault().getTable("limelight-one");
        
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
    // LimelightHelpers.SetFiducialIDFiltersOverride(limelightName3, getCoralTags(alliance));
    // LimelightHelpers.SetFiducialIDFiltersOverride(limelightName5, getCoralTags(alliance));
  }
  
  public double LimelightMountAngle() {
    
    return java.lang.Math.atan((TARGET_MOUNT_HEIGHT - LIMELIGHT_HEIGHT) / Ta);
  }
  
  
  public double getDistance() {
    Rotation2d angleToGoal = Rotation2d.fromDegrees(LimelightMountAngle())
    .plus(Rotation2d.fromDegrees(LimelightHelpers.getTX("limelight-five")));
    double distance = (TARGET_MOUNT_HEIGHT - LIMELIGHT_HEIGHT/*height of the limelight camera off the ground, make sure to measure this as often as possible */) / angleToGoal.getTan();
    return distance;
  }
  
  public double getRotation() {
    double cameraLensHorizontalOffset = LimelightHelpers.getTX("limelight-five") / getDistance();
    double realHorizontalOffset = Math.atan(cameraLensHorizontalOffset / getDistance());
    double rotationError = Math.atan(realHorizontalOffset / getDistance());
    return rotationError;
  }
  
/**
 * Command to align the robot to a coral reef using a direct approach.
 * This method uses a completely different strategy - instead of continuous feedback,
 * it takes a single measurement and executes a direct path to the target.
 * 
 * @param alignmentDirection "left" or "right" to select which limelight to use
 * @return Command to align to the coral reef
 */
public Command alignToCoralReef(String alignmentDirection) {
  if (alignmentDirection != "left") {
    alignmentDirection = "right";
  }
  final String selectedLimelight = alignmentDirection == "left" ? limelightName3 : limelightName5;
  
  // Target distance in meters (1 foot = 0.3048 meters)
  final double TARGET_DISTANCE = 0.3048;
  
  return new Command() {
    private boolean isFinished = false;
    private double initialTx = 0;
    private double initialDistance = 0;
    private double rotationDuration = 0;
    private double forwardDuration = 0;
    private long startTime = 0;
    private int stage = 0; // 0=init, 1=rotate, 2=move forward/back, 3=done
    
    @Override
    public void initialize() {
      // Wait until we have a valid target
      if (!LimelightHelpers.getTV(selectedLimelight)) {
        System.out.println("No valid target found. Aborting alignment.");
        isFinished = true;
        return;
      }
      
      // Take initial measurements
      initialTx = LimelightHelpers.getTX(selectedLimelight);
      
      double[] robotPose = LimelightHelpers.getBotPose_TargetSpace(selectedLimelight);
      if (robotPose != null && robotPose.length >= 3) {
        initialDistance = Math.abs(robotPose[2]);
      } else {
        System.out.println("Could not get robot pose. Aborting alignment.");
        isFinished = true;
        return;
      }
      
      // Log initial state
      System.out.println("Starting alignment with tx=" + initialTx + ", distance=" + initialDistance);
      
      // Calculate how long to rotate and move
      // Use a fixed rotation rate of 0.3 rad/s
      double rotationRate = 0.3; // radians per second
      rotationDuration = Math.abs(Math.toRadians(initialTx) / rotationRate);
      
      // Use a fixed forward speed of 0.3 m/s
      double forwardSpeed = 0.3; // meters per second
      forwardDuration = Math.abs(initialDistance - TARGET_DISTANCE) / forwardSpeed;
      
      // Cap durations to reasonable values
      rotationDuration = Math.min(rotationDuration, 3.0); // max 3 seconds of rotation
      forwardDuration = Math.min(forwardDuration, 3.0);   // max 3 seconds of forward movement
      
      System.out.println("Planned rotation for " + rotationDuration + " seconds");
      System.out.println("Planned forward movement for " + forwardDuration + " seconds");
      
      startTime = System.currentTimeMillis();
      stage = 1; // Start with rotation
    }
    
    @Override
    public void execute() {
      long currentTime = System.currentTimeMillis();
      double elapsedSeconds = (currentTime - startTime) / 1000.0;
      
      if (stage == 1) {
        // Rotation stage
        if (elapsedSeconds < rotationDuration) {
          // Still rotating
          double rotationRate = initialTx > 0 ? -0.3 : 0.3; // Rotate opposite to tx
          drivetrain.setControl(
              new SwerveRequest.RobotCentric()
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                  .withVelocityX(0)
                  .withVelocityY(0)
                  .withRotationalRate(rotationRate)
          );
        } else {
          // Done rotating, move to next stage
          System.out.println("Rotation complete, moving to forward/backward stage");
          startTime = System.currentTimeMillis(); // Reset timer for next stage
          stage = 2;
        }
      } else if (stage == 2) {
        // Forward/backward movement stage
        if (elapsedSeconds < forwardDuration) {
          // Still moving
          double forwardSpeed = initialDistance > TARGET_DISTANCE ? 0.3 : -0.3;
          drivetrain.setControl(
              new SwerveRequest.RobotCentric()
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                  .withVelocityX(forwardSpeed)
                  .withVelocityY(0)
                  .withRotationalRate(0)
          );
        } else {
          // Done moving, finish command
          System.out.println("Forward/backward movement complete, alignment finished");
          drivetrain.setControl(
              new SwerveRequest.RobotCentric()
                  .withVelocityX(0)
                  .withVelocityY(0)
                  .withRotationalRate(0)
          );
          stage = 3;
          isFinished = true;
        }
      }
    }
    
    @Override
    public void end(boolean interrupted) {
      // Stop the robot
      drivetrain.setControl(
          new SwerveRequest.RobotCentric()
              .withVelocityX(0)
              .withVelocityY(0)
              .withRotationalRate(0)
      );
      
      if (interrupted) {
        System.out.println("Alignment was interrupted");
      } else {
        System.out.println("Alignment completed successfully");
      }
    }
    
    @Override
    public boolean isFinished() {
      return isFinished;
    }
  };
}
  
  public Command RobotToLeftCoralStation() {
    // Create a SwerveRequest object for the desired motion
    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withVelocityX(0.25) // Forward speed in meters per second
    .withVelocityY(0.25) // Side to side speed
    .withRotationalRate(1); // Rotation in radians per second
    
    
    return Commands.sequence(
    Commands.runOnce(() -> drivetrain.setControl(drive)),
    // Wait until we reach the target
    Commands.waitUntil(() -> Ta > 5),
    // Stop the robot
    Commands.runOnce(() -> drivetrain.setControl(
    new SwerveRequest.FieldCentric()
    .withVelocityX(0)
    .withVelocityY(0)
    .withRotationalRate(0)))
    );
  }
  
  public Command RobotToRightCoralStation() {
    
    return Commands.sequence(
    
    
    
    );
    
  }
  
  
}










/* 
// simple proportional turning control with Limelight.
// "proportional control" is a control algorithm in which the output is proportional to the error.
// in this case, we are going to return an angular velocity that is proportional to the 
// "tx" value from the Limelight.
double limelight_aim_proportional()
{    
// kP (constant of proportionality)
// this is a hand-tuned number that determines the aggressiveness of our proportional control loop
// if it is too high, the robot will oscillate around.
// if it is too low, the robot will never reach its target
// if the robot never turns in the correct direction, kP should be inverted.
double kP = .035;

// tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
// your limelight 3 feed, tx should return roughly 31 degrees.
double targetingAngularVelocity = LimelightHelpers.getTX("limelight-three") * kP;


// convert to radians per second for our drive method
targetingAngularVelocity *= drivetrain.kMaxAngularSpeed;

//invert since tx is positive when the target is to the right of the crosshair
targetingAngularVelocity *= -1.0;

return targetingAngularVelocity;
}

// simple proportional ranging control with Limelight's "ty" value
// this works best if your Limelight's mount height and target mount height are different.
// if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
double limelight_range_proportional()
{    
double kP = .1;
double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
targetingForwardSpeed *= drivetrain.kMaxSpeed;
targetingForwardSpeed *= -1.0;
return targetingForwardSpeed;
}

private void drive(boolean fieldRelative) {
// Get the x speed. We are inverting this because Xbox controllers return
// negative values when we push forward.
var xSpeed =
-m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02))
* drivetrain.kMaxSpeed;

// Get the y speed or sideways/strafe speed. We are inverting this because
// we want a positive value when we pull to the left. Xbox controllers
// return positive values when you pull to the right by default.
var ySpeed =
-m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02))
* drivetrain.kMaxSpeed;

// Get the rate of angular rotation. We are inverting this because we want a
// positive value when we pull to the left (remember, CCW is positive in
// mathematics). Xbox controllers return positive values when you pull to
// the right by default.
var rot =
-m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
* drivetrain.kMaxAngularSpeed;

// while the A-button is pressed, overwrite some of the driving values with the output of our limelight methods
if(m_controller.getAButton())
{
final var rot_limelight = limelight_aim_proportional();
rot = rot_limelight;

final var forward_limelight = limelight_range_proportional();
xSpeed = forward_limelight;

//while using Limelight, turn off field-relative driving.
fieldRelative = false;
}

m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
}

}



*/
