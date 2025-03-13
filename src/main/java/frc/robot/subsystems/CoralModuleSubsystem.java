package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralModuleSubsystem extends SubsystemBase {
  private static final class Constants {
    private static final int MOTOR_ID = -1;
  }

  private final TalonFX motor;
  private StatusSignal<Voltage> motorSupplyVoltage;
  private boolean hasCoral;

  public CoralModuleSubsystem() {
    this.motor = new TalonFX(Constants.MOTOR_ID);
    this.motorSupplyVoltage = this.motor.getSupplyVoltage();
  }

  @Override
  public void periodic() {
    Voltage lastMotorSupplyVoltage = this.motorSupplyVoltage.getValue();
    this.motorSupplyVoltage.refresh();
    Voltage newMotorSupplyVoltage = this.motorSupplyVoltage.getValue();

    if (lastMotorSupplyVoltage.in(Volts) - newMotorSupplyVoltage.in(Volts) > 2) { // rnadom number here for now
      this.hasCoral = true;
    }
  }

  public boolean hasCoral() {
    return this.hasCoral;
  }

  public void resetCoralState() {
    this.hasCoral = false;
  }
}
