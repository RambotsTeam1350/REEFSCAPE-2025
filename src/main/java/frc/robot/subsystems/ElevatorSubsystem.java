package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import com.ctre.phoenix6.hardware.core.CoreTalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX elevatorMotorOne;
    private final TalonFX elevatorMotorTwo;
    private final double gearBoxRatio = 12;
    public final StatusSignal<Angle> position1;
    public final StatusSignal<Angle> position2;

    public ElevatorSubsystem() {
        elevatorMotorOne = new TalonFX(16);
        elevatorMotorTwo = new TalonFX(17);
        position1 = elevatorMotorOne.getPosition();
        position2 = elevatorMotorTwo.getPosition();

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = 4.8; // P value: Position
        cfg.Slot0.kI = 0; // I value: Integral
        cfg.Slot0.kD = 0.1; // D value: Derivative
        cfg.Slot0.kV = 0.12; // V value: Velocity
        //cfg.Slot0.kG = 0.1; // G value: Feedforward
        cfg.Slot0.kA = 0.01; // A value: Acceleration
        cfg.Slot0.kS = 0.25; // S value: Soft Limit
    
        MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    mm.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    mm.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
    
    elevatorMotorOne.getConfigurator().apply(cfg);
    elevatorMotorTwo.getConfigurator().apply(cfg);

}

private double degreesWithGearBoxRatio(double degrees) {
    // Assuming 2048 units per revolution and a gear ratio of 9:1
    //double unitsPerRevolution = 2048;
    return (degrees / 360.0) * gearBoxRatio;
}

public void periodic() {
    BaseStatusSignal.refreshAll(position1, position2);
    //System.out.println(position1.getValueAsDouble() + " elevator motor 1");
    //System.out.println(position2.getValueAsDouble() + " elevator motor 2");
}

public Command L1Command() {

    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    return Commands.sequence(
        
        Commands.runOnce (() -> elevatorMotorOne.setControl(m_request.withPosition(0.5))),
        Commands.runOnce (() -> elevatorMotorTwo.setControl(m_request.withPosition(-0.5)))
    );
}

public Command L2Command() {
    // create a Motion Magic request, voltage output
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
   //System.out.println();
    return Commands.sequence(
       
    // set target position to 100 rotations
        Commands.runOnce (() -> elevatorMotorOne.setControl(m_request.withPosition(15))),
        Commands.runOnce (() -> elevatorMotorTwo.setControl(m_request.withPosition(-15)))
    );
}

public Command L3Command() {
final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
return Commands.sequence(
    
    Commands.runOnce (() -> elevatorMotorOne.setControl(m_request.withPosition(30))),
    Commands.runOnce (() -> elevatorMotorTwo.setControl(m_request.withPosition(-30)))
);
}

public Command L4Command() {
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    return Commands.sequence(
        
        Commands.runOnce (() -> elevatorMotorOne.setControl(m_request.withPosition(45))),
        Commands.runOnce (() -> elevatorMotorTwo.setControl(m_request.withPosition(-45)))

    );

}

public Command LBargeCommand() {

    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    return Commands.sequence(
        
        Commands.runOnce (() -> elevatorMotorOne.setControl(m_request.withPosition(60))),
        Commands.runOnce (() -> elevatorMotorTwo.setControl(m_request.withPosition(-60)))
    );
}

}