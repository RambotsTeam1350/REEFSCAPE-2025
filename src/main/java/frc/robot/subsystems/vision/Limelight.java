package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.LimelightHelpers.RawFiducial;

public class Limelight extends SubsystemBase {
  public Limelight() {
  }

  public void config() {
    LimelightHelpers.setCropWindow("", -0.5, 0.5, -0.5, 0.5);
    LimelightHelpers.setCameraPose_RobotSpace(
        "",
        Meters.convertFrom(12.75, Inches),
        0,
        Meters.convertFrom(6, Inches),
        0,
        0,
        0);
    LimelightHelpers.SetFiducialIDFiltersOverride("", new int[] { 1, 4 });
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("txnc", );
  }
}
