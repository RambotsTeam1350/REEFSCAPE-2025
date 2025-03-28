package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
  private final TalonFX rightMotor = new TalonFX(18);
  private final TalonFX leftMotor = new TalonFX(19);

  private StatusSignal<Voltage> motorSupplyVoltage;
  //private StatusSignal isDetected;
  private boolean hasCoral;

private final LEDCandle LEDCandle = new LEDCandle(); 

  public CoralModuleSubsystem() {

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.Slot0.kP = 4.8; // P value: Position
    cfg.Slot0.kI = 0; // I value: Integral
    cfg.Slot0.kD = 0.1; // D value: Derivative
    cfg.Slot0.kV = 0.12; // V value: Velocity
    //cfg.Slot0.kG = 0.1; // G value: Feedforward
    cfg.Slot0.kA = 0.01; // A value: Acceleration
    cfg.Slot0.kS = 0.25; // S value: Soft Limit

    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps NOTE THIS IS THE SPEED IT WILL GO, WANT SLOWER, MAKE THIS SLOWER
    mm.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    mm.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    leftMotor.getConfigurator().apply(cfg);
    rightMotor.getConfigurator().apply(cfg);


    this.CANrange = new CANrange(23);
    this.motorSupplyVoltage = leftMotor.getSupplyVoltage();
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
        Commands.runOnce(() -> leftMotor.set(0.5)).alongWith(LEDCandle.LEDYellow()),
        Commands.waitUntil(() -> CANrange.getIsDetected(true).getValue() == true),
        Commands.runOnce(() -> leftMotor.set(0.2)).alongWith(LEDCandle.LEDRed()),
        Commands.waitUntil(() -> CANrange.getIsDetected(true).getValue() == false),
        Commands.runOnce(() -> leftMotor.set(0)).alongWith(LEDCandle.LEDGreen())
    );
    
    } 

    public Command deliverCoral() {        
        return Commands.sequence(
            Commands.runOnce(() -> leftMotor.set(0.2)).alongWith(LEDCandle.LEDOff()),
            Commands.waitSeconds(2),
            Commands.runOnce(() -> leftMotor.set(0))
        );
    } 
}
