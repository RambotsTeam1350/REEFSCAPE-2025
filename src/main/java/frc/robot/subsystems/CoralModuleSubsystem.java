package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.LEDCandle;

public class CoralModuleSubsystem extends SubsystemBase {
  

  private final CANrange CANrange;
  private final TalonFX motor = new TalonFX(24);
  private StatusSignal<Voltage> motorSupplyVoltage;
  //private StatusSignal isDetected;
  private boolean hasCoral;

private final LEDCandle LEDCandle = new LEDCandle(); 

  public CoralModuleSubsystem() {
    this.CANrange = new CANrange(23);
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


 /*
  public boolean hasCoral() {
  [CANrange variable].[sensor?] 
  }

  */

  public boolean hasCoral() {
    return this.hasCoral;
  }

  public void resetCoralState() {
    this.hasCoral = false;
  }

   /* public Command IntakeCoralCommand() {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        return Commands.sequence(
            Commands.runOnce (() -> this.resetCoralState()),
            Commands.runOnce (() -> this.motor.set(0.5)),
            Commands.waitUntil(this::hasCoral),
            Commands.runOnce (() -> this.motor.set(0.2)),
            Commands.waitSeconds(2)
        );
    }

   */ 

 public StatusSignal getIsDetected(boolean refresh) {

   return CANrange.getIsDetected(refresh);
   //
  }


  
     public Command IntakeCoralCommand() {
    
    return Commands.sequence(
        Commands.runOnce(() -> motor.set(0.5)).alongWith(LEDCandle.LEDYellow()),
        Commands.waitUntil(() -> CANrange.getIsDetected(true).getValue() == true),
        Commands.runOnce(() -> motor.set(0.2)).alongWith(LEDCandle.LEDRed()),
        Commands.waitUntil(() -> CANrange.getIsDetected(true).getValue() == false),
        Commands.runOnce(() -> motor.set(0)).alongWith(LEDCandle.LEDGreen())
    );
    
    } 
    public Command deliverCoral() {
        
        return Commands.sequence(
            Commands.runOnce(() -> motor.set(0.2)).alongWith(LEDCandle.LEDOff()),
            Commands.waitSeconds(2),
            Commands.runOnce(() -> motor.set(0))
        );
    } 
}
