package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.LimelightHelpers.RawFiducial;

public class LimelightSubsystem extends SubsystemBase {
  public LimelightSubsystem() {
    config();
  }

  public void config() {
    // LimelightHelpers.setCropWindow("", -0.5, 0.5, -0.5, 0.5);
    LimelightHelpers.setCameraPose_RobotSpace(
        "",
        Meters.convertFrom(12.75, Inches),
        0,
        0.195,
        0,
        0,
        0);
    LimelightHelpers.SetFiducialIDFiltersOverride("", new int[] { 5 });
  }

  @Override
  public void periodic() {
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
    for (RawFiducial fiducial : fiducials) {
      int id = fiducial.id; // Tag ID
      double txnc = fiducial.txnc; // X offset (no crosshair)
      double tync = fiducial.tync; // Y offset (no crosshair)
      double ta = fiducial.ta; // Target area
      double distToCamera = fiducial.distToCamera; // Distance to camera
      double distToRobot = fiducial.distToRobot; // Distance to robot
      double ambiguity = fiducial.ambiguity; // Tag pose ambiguity
      SmartDashboard.putNumber("id", id);
      SmartDashboard.putNumber("txnc", txnc);
      SmartDashboard.putNumber("tync", tync);
      SmartDashboard.putNumber("ta", ta);
      SmartDashboard.putNumber("distToCamera", distToCamera);
      SmartDashboard.putNumber("distToRobot", distToRobot);
      SmartDashboard.putNumber("ambiguity", ambiguity);
    }
  }
}
