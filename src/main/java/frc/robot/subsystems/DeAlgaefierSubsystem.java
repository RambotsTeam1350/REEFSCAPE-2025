package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DeAlgaefierSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(25);
    private StatusSignal<Angle> motorPosition;
    private final double gearBoxRatio = 9;

    public DeAlgaefierSubsystem() {
       
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = 4.8; // P value: Position
        cfg.Slot0.kI = 0; // I value: Integral
        cfg.Slot0.kD = 0.1; // D value: Derivative
        cfg.Slot0.kV = 0.12; // V value: Velocity
        //cfg.Slot0.kG = 0.1; // G value: Feedforward
        cfg.Slot0.kA = 0.01; // A value: Acceleration
        cfg.Slot0.kS = 0.25; // S value: Soft Limit

        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 80 rps NOTE: this speed is defaulty 80, its 20 to try and run slower
        mm.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        mm.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        motor.getConfigurator().apply(cfg);
    }

    private double degreesToEncoderUnits(double degrees) {
        // Assuming 2048 units per revolution and a gear ratio of 9:1
        double unitsPerRevolution = 2048;
        return (degrees / 360.0) * gearBoxRatio;
    }

    public Command deAlgaefierUp() {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        return Commands.sequence(

            Commands.runOnce(() -> motor.setControl(new MotionMagicVoltage(degreesToEncoderUnits(180))))
        );
    }

    public Command deAlgaefierDown() {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        return Commands.sequence(

            Commands.runOnce(() -> motor.setControl(new MotionMagicVoltage(degreesToEncoderUnits(0))))
        );
    }


    }

