package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralModuleSubsystem extends SubsystemBase {
  private static final class Constants {
    private static final int motorId = -1;
  }

  private final SparkFlex motor;

  public CoralModuleSubsystem() {
    this.motor = new SparkFlex(Constants.motorId, MotorType.kBrushless);

  }
}
