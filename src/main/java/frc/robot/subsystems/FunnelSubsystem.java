package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FunnelSubsystem extends SubsystemBase {

    private final TalonFX motor1 = new TalonFX(18);
    private final TalonFX motor2 = new TalonFX(19);
    private final double gearBoxRatio = 9;
    private StatusSignal<Angle> motorPosition1 = motor1.getPosition();
    private StatusSignal<Angle> motorPosition2 = motor2.getPosition();

    public FunnelSubsystem() {
    

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = 4.8; // P value: Position
        cfg.Slot0.kI = 0.2; // I value: Integral
        cfg.Slot0.kD = 0.1; // D value: Derivative
        cfg.Slot0.kV = 0.09; // V value: Velocity
        //cfg.Slot0.kG = 0.1; // G value: Feedforward
        cfg.Slot0.kA = 0.15; // A value: Acceleration
        cfg.Slot0.kS = 0.25; // S value: Soft Limit

        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps NOTE THIS IS THE SPEED IT WILL GO, WANT SLOWER, MAKE THIS SLOWER
        mm.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        mm.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        motor1.getConfigurator().apply(cfg);
        motor2.getConfigurator().apply(cfg);
}

private double degreesToEncoderUnits(double degrees) {
    // Assuming 2048 units per revolution and a gear ratio of 9:1
    double unitsPerRevolution = 2048;
    return (degrees / 360.0) * gearBoxRatio;
}

public void periodic() {
    BaseStatusSignal.refreshAll(this.motorPosition1, this.motorPosition2);
    //System.out.println(motorPosition1.getValueAsDouble() + " Funnel motor 1");
    //System.out.println(motorPosition2.getValueAsDouble() + " Funnel motor 2");
}
public Command funnelClose() {
    // create a Motion Magic request, voltage output
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    //System.out.println();
    return Commands.sequence(
        
    // set target position to 100 rotations
        Commands.runOnce (() -> motor1.setControl(m_request.withPosition(-0.32))),
        Commands.runOnce (() -> motor2.setControl(m_request.withPosition(0.34)))
    );
}
    
public Command funnelOpen() {

    // create a Motion Magic request, voltage output
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    //System.out.println();
    return Commands.sequence(
        
    // set target position to 100 rotations
        Commands.runOnce (() -> motor1.setControl(m_request.withPosition(-0.8))),
        Commands.runOnce (() -> motor2.setControl(m_request.withPosition(1.06)))
    );
}

}