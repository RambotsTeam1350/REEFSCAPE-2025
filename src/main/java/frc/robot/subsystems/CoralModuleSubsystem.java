package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.LEDCandle;

public class CoralModuleSubsystem extends SubsystemBase {
    private final CANrange canRange;
    private final TalonFX motor1 = new TalonFX(20);
    private final TalonFX motor2 = new TalonFX(21);
    private final StatusSignal<Boolean> detectionSignal;
    private final LEDCandle LEDCandle = new LEDCandle();

    public CoralModuleSubsystem() {
        canRange = new CANrange(23); // Adjust ID as needed
        
      
        CANrangeConfiguration config = new CANrangeConfiguration();
        config.ProximityParams.ProximityThreshold = 0.15;
        // config.ProximityParams.ProximityHysteresis = 0.04;

        // Apply configuration
      canRange.getConfigurator().apply(config);

        detectionSignal = canRange.getIsDetected();
    }

    @Override
    public void periodic() {
        detectionSignal.refresh();
    }

    public boolean isObjectDetected() {
        return detectionSignal.getValue();
    }

    public Command IntakeCoralCommand() {
        return Commands.sequence(
            // Start motors
            Commands.runOnce(() -> {
                motor1.set(-0.4);
                motor2.set(0.4);
            }).alongWith(LEDCandle.LEDYellow()),
            
            // Wait for object detection
            Commands.waitUntil(this::isObjectDetected),
            
            // Slow motors when detected
            Commands.runOnce(() -> {
                motor1.set(-0.15);
                motor2.set(0.15);
            }).alongWith(LEDCandle.LEDRed()),
            
            // Wait until object passes
            Commands.waitUntil(() -> !isObjectDetected()),
            
            // Stop motors
            Commands.runOnce(() -> {
                motor1.set(0);
                motor2.set(0);
            }).alongWith(LEDCandle.LEDGreen())
        );
    }

    public Command deliverCoral() {
        return Commands.sequence(
            Commands.runOnce(() -> motor1.set(-0.15)).alongWith(LEDCandle.LEDOff()),
            Commands.runOnce(() -> motor2.set(0.15)),
            Commands.waitSeconds(1),
            Commands.runOnce(() -> motor1.set(0)),
            Commands.runOnce(() -> motor2.set(0))
        );
    }
}
