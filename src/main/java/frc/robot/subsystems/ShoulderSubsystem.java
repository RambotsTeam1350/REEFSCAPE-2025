package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShoulderSubsystem extends SubsystemBase {
    private final TalonFX shoulderMotor;
    private final double gearBoxRatio = 9;
    public final StatusSignal<Angle> position;

public ShoulderSubsystem() {

shoulderMotor = new TalonFX(18);
position = shoulderMotor.getPosition();

TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.Slot0.kP = 4.8; // P value: Position
    cfg.Slot0.kI = 0; // I value: Integral
    cfg.Slot0.kD = 0.1; // D value: Derivative
    cfg.Slot0.kV = 0.12; // V value: Velocity
    //cfg.Slot0.kG = 0.1; // G value: Feedforward
    cfg.Slot0.kA = 0.01; // A value: Acceleration
    cfg.Slot0.kS = 0.25; // S value: Soft Limit

MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 120; // Target cruise velocity of 80 rps
    mm.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    mm.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    shoulderMotor.getConfigurator().apply(cfg);
}

private double degreesWithGearBoxRatio(double degrees) {
    // Assuming 2048 units per revolution and a gear ratio of 20/1
    //double unitsPerRevolution = 2048;
    return (degrees / 360.0) * gearBoxRatio;
}

public void periodic() {
        BaseStatusSignal.refreshAll(position);
    System.out.println(position.getValueAsDouble() + " shoulder motor");
}

public Command ShoulderToRestPosition() {

    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    return Commands.sequence(

        Commands.runOnce (() -> shoulderMotor.setControl(m_request.withPosition(0)))
    );
}

public Command ShoulderToLevel1() {

    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    return Commands.sequence(

        Commands.runOnce (() -> shoulderMotor.setControl(m_request.withPosition(-11.8)))
    );
}

public Command ShoulderToLevel2() {

    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    return Commands.sequence(
        
        Commands.runOnce(() -> shoulderMotor.setControl(m_request.withPosition(-11)))
    );
}

public Command ShoulderToLevel3() {

    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    return Commands.sequence(
        
        Commands.runOnce(() -> shoulderMotor.setControl(m_request.withPosition(-2)))
    );
}

public Command ShoulderToLevel4() {

    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    return Commands.sequence(
        
        Commands.runOnce(() -> shoulderMotor.setControl(m_request.withPosition(-1)))
    );
}

public Command ShoulderToCoralStation() {

    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    return Commands.sequence(
        
        Commands.runOnce(() -> shoulderMotor.setControl(m_request.withPosition(7)))
    );
}

public Command ShoulderToLevelBarge() {

    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    return Commands.sequence(
        
        Commands.runOnce(() -> shoulderMotor.setControl(m_request.withPosition(12)))
    );
}
// there may be other commands for other angles of the shoulder added later

}
