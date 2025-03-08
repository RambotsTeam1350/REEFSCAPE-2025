package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.CAN;

import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.SparkMax;
//import com.revrobotics.CANSparkBase.ControlType;
//import com.revrobotics.CANSparkBase.IdleMode;
//import com.revrobotics.CANSparkLowLevel.MotorType;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralModuleSubsystem {
    private final TalonFX moduleMotor;
    private final double gearBoxRatio = 9;
    public final StatusSignal<Angle> position;

    public CoralModuleSubsystem() {

        moduleMotor = new TalonFX(19);
        position = moduleMotor.getPosition();

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
    
    moduleMotor.getConfigurator().apply(cfg);

    }

    public void periodic() {
           // BaseStatusSignal.refreshAll(position);
        //System.out.println(position.getValueAsDouble() + " module motor");
    }


    public Command IntakeCoral() {

        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        return Commands.sequence(

            Commands.runOnce (() -> moduleMotor.setControl(m_request.withPosition(50))) // encoder value

        );
    }
}
