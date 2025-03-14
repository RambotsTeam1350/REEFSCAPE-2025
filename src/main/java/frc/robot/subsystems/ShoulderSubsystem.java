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

public class ShoulderSubsystem extends SubsystemBase {
    public static final class Constants {
        public static final int MOTOR_ID = 20;
        public static final double GEARBOX_RATIO = 125;

        public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs().withKP(4.8).withKI(0).withKD(0.1)
                .withKV(0.12).withKA(0.01).withKS(0.25);
        public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(80).withMotionMagicAcceleration(160).withMotionMagicJerk(1600);
        public static final TalonFXConfiguration TALON_FX_CONFIGURATION = new TalonFXConfiguration()
                .withSlot0(SLOT0_CONFIGS).withMotionMagic(MOTION_MAGIC_CONFIGS).withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    }

    private final TalonFX motor;

    private StatusSignal<Angle> motorPosition;

    private final MotionMagicVoltage motorMotionMagicVoltage;


    public ShoulderSubsystem() {
        this.motor = new TalonFX(Constants.MOTOR_ID);
        
        this.motorPosition = this.motor.getPosition();

        this.motor.getConfigurator().apply(Constants.TALON_FX_CONFIGURATION);

        this.motorMotionMagicVoltage = new MotionMagicVoltage(0);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(this.motorPosition);
        System.out.println("Shoulder Position: " + motorPosition.getValueAsDouble());  
    }

    public Command restPositionCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> this.motor.setControl(this.motorMotionMagicVoltage.withPosition(0))));
    }

    public Command level1Command() {
        return Commands.sequence(
                Commands.runOnce(() -> this.motor.setControl(this.motorMotionMagicVoltage.withPosition(11.8)))

                
                
                );

    }

    public Command level2Command() {
        return Commands.sequence(
                Commands.runOnce(() -> this.motor.setControl(this.motorMotionMagicVoltage.withPosition(159.72))));
    }

    public Command level3Command() {
        return Commands.sequence(
                Commands.runOnce(() -> this.motor.setControl(this.motorMotionMagicVoltage.withPosition(153.71))));
    }

    public Command level4Command() {
        return Commands.sequence(
                Commands.runOnce(() -> this.motor.setControl(this.motorMotionMagicVoltage.withPosition(247.222))));
    }

    public Command coralStationCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> this.motor.setControl(this.motorMotionMagicVoltage.withPosition(247.222))));
    }

    public Command levelBargeCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> this.motor.setControl(this.motorMotionMagicVoltage.withPosition(12))));
    }
}
