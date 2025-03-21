package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Arrays;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LimelightConfig;
import frc.robot.subsystems.vision.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.vision.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.vision.LimelightHelpers.RawFiducial;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LimelightSubsystem extends SubsystemBase {
  public final LimelightConfig limelightConfig;
  public final String limelightName;

  public RawFiducial[] fiducials;
  public LimelightResults limelightResults;

  private final int[] CORAL_TAGS_RED = {6, 7, 8, 9, 10, 11};
  private final int[] CORAL_TAGS_BLUE = {17, 18, 19, 20, 21, 22};

  public final int[] getCoralTags(Alliance alliance) {
    if (alliance == Alliance.Red) {
      return CORAL_TAGS_RED;
    } else {
      return CORAL_TAGS_BLUE;
    }
  }

  public LimelightSubsystem(LimelightConfig limelightConfig, boolean filterAprilTags) {
    this.limelightConfig = limelightConfig;
    this.limelightName = this.limelightConfig.name();

    LimelightHelpers.setCameraPose_RobotSpace(
        this.limelightName,
        this.limelightConfig.forward(),
        this.limelightConfig.side(),
        this.limelightConfig.up(),
        this.limelightConfig.roll(),
        this.limelightConfig.pitch(),
        this.limelightConfig.yaw());

    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
    if (filterAprilTags) {
      System.out.println("FILTERING CORAL TAGS");
      LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, getCoralTags(alliance));
    }
  }

  public static class NoSuchTargetException extends RuntimeException {
    public NoSuchTargetException(String message) {
      super(message);
    }
  }

  @Override
  public void periodic() {
    this.limelightResults = LimelightHelpers.getLatestResults(this.limelightName);
    
    // Output the number of detected fiducials
    int numFiducials = (this.limelightResults != null && this.limelightResults.targets_Fiducials != null) ? 
        this.limelightResults.targets_Fiducials.length : 0;
    SmartDashboard.putNumber("num" + this.limelightName, numFiducials);
    
    // Output the currently detected April tag ID
    int currentTagId = getCurrentAprilTagId();
    SmartDashboard.putNumber("Current April Tag ID", currentTagId);
    
    // Output all detected April tag IDs
    int[] allTagIds = getAllCurrentAprilTagIds();
    if (allTagIds.length > 0) {
      StringBuilder allTags = new StringBuilder();
      for (int id : allTagIds) {
        allTags.append(id).append(", ");
      }
      // Remove the trailing comma and space
      if (allTags.length() > 2) {
        allTags.setLength(allTags.length() - 2);
      }
      SmartDashboard.putString("All Detected April Tags", allTags.toString());
    } else {
      SmartDashboard.putString("All Detected April Tags", "None");
    }
    
    // Uncomment to use raw fiducials if needed
    // this.fiducials = LimelightHelpers.getRawFiducials(this.limelightName);
    
    // for (RawFiducial fiducial : fiducials) {
    // int id = fiducial.id; // Tag ID
    // double txnc = fiducial.txnc; // X offset (no crosshair)
    // double tync = fiducial.tync; // Y offset (no crosshair)
    // double ta = fiducial.ta; // Target area
    // double distToCamera = fiducial.distToCamera; // Distance to camera
    // double distToRobot = fiducial.distToRobot; // Distance to robot
    // double ambiguity = fiducial.ambiguity; // Tag pose ambiguity
    // SmartDashboard.putNumber("id", id);
    // SmartDashboard.putNumber("txnc", txnc);
    // SmartDashboard.putNumber("tync", tync);
    // SmartDashboard.putNumber("ta", ta);
    // SmartDashboard.putNumber("distToCamera", distToCamera);
    // SmartDashboard.putNumber("distToRobot", distToRobot);
    // SmartDashboard.putNumber("ambiguity", ambiguity);
    // }
  }

  public void setFiducialIDFiltersOverride(int[] ids) {
    LimelightHelpers.SetFiducialIDFiltersOverride(this.limelightName, ids);
  }

  public RawFiducial getRawFiducialWithId(int id) {
    for (RawFiducial fiducial : fiducials) {
      if (fiducial.id != id) {
        continue;
      }

      return fiducial;
    }
    throw new NoSuchTargetException("No target with ID " + id + " is in view!");
  }

  public LimelightTarget_Fiducial getTargetFiducialWithId(int id) {
    for (LimelightTarget_Fiducial fiducial : limelightResults.targets_Fiducials) {
      if (fiducial.fiducialID != (double) id) {
        continue;
      }

      return fiducial;
    }

    throw new NoSuchTargetException("No target with ID " + id + " is in view!");
  }

  public RawFiducial getFiducialWithIdFirstMatch(int[] ids) {
    for (RawFiducial fiducial : fiducials) {
      if (!Arrays.stream(ids).anyMatch(i -> i == fiducial.id)) {
        continue;
      }

      return fiducial;
    }
    throw new NoSuchTargetException("No target with ID " + ids + "is in view!");
  }

  /**
   * Gets the first detected fiducial (April tag) from the Limelight.
   * 
   * @return The first detected fiducial
   * @throws NoSuchTargetException if no fiducials are detected
   */
  public LimelightTarget_Fiducial getFiducial() {
    if (this.limelightResults == null || this.limelightResults.targets_Fiducials == null 
        || this.limelightResults.targets_Fiducials.length == 0) {
      throw new NoSuchTargetException("No fiducials are in view!");
    }
    return this.limelightResults.targets_Fiducials[0];
  }
  
  /**
   * Gets the ID of the first detected fiducial (April tag) from the Limelight.
   * 
   * @return The ID of the first detected fiducial, or -1 if no fiducials are detected
   */
  public int getCurrentAprilTagId() {
    try {
      LimelightTarget_Fiducial fiducial = getFiducial();
      return (int) fiducial.fiducialID;
    } catch (NoSuchTargetException e) {
      return -1;
    }
  }
  
  /**
   * Gets all currently detected fiducial (April tag) IDs from the Limelight.
   * 
   * @return An array of detected fiducial IDs, or an empty array if no fiducials are detected
   */
  public int[] getAllCurrentAprilTagIds() {
    if (this.limelightResults == null || this.limelightResults.targets_Fiducials == null 
        || this.limelightResults.targets_Fiducials.length == 0) {
      return new int[0];
    }
    
    int[] tagIds = new int[this.limelightResults.targets_Fiducials.length];
    for (int i = 0; i < this.limelightResults.targets_Fiducials.length; i++) {
      tagIds[i] = (int) this.limelightResults.targets_Fiducials[i].fiducialID;
    }
    return tagIds;
  }

}
