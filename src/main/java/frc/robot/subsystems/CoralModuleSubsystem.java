package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralModuleSubsystem extends SubsystemBase {
  private static final class Constants {
    private static final int MOTOR_ID = -1;
  }

  private final TalonFX motor;

  private StatusSignal<Voltage> motorSupplyVoltage;

  private final MotionMagicVoltage motorMotionMagicVoltage;

  private boolean hasCoral;

  public CoralModuleSubsystem() {
    this.motor = new TalonFX(Constants.MOTOR_ID);
    this.motorSupplyVoltage = this.motor.getSupplyVoltage();
    this.motorMotionMagicVoltage = new MotionMagicVoltage(0);
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

  public Command intakeCoral() {
    return Commands.sequence(
        Commands.runOnce(() -> motor.setControl(this.motorMotionMagicVoltage.withPosition(15))));
  }

  public Command deliverCoral() {
    return Commands.sequence(
        Commands.runOnce(() -> motor.setControl(this.motorMotionMagicVoltage.withPosition(0))));
  }
}
