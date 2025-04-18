package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    public static final class Constants {
        public static final int MOTOR_1_ID = 15;
        public static final int MOTOR_2_ID = 14;

        public static final double GEARBOX_RATIO = 135;
        public static final double ROTATION_AMOUNT = 0.375;

        public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs().withKP(4.8).withKI(0).withKD(0.1)
                .withKV(0.12).withKA(0.01).withKS(0.25);
        public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(40).withMotionMagicAcceleration(160).withMotionMagicJerk(1600);
        public static final TalonFXConfiguration TALON_FX_CONFIGURATION = new TalonFXConfiguration()
                .withSlot0(SLOT0_CONFIGS).withMotionMagic(MOTION_MAGIC_CONFIGS);
    }

    private final TalonFX motor1;
    private final TalonFX motor2;

    private final MotionMagicVoltage motor1MotionMagicVoltageRequest;
    private final MotionMagicVoltage motor2MotionMagicVoltageRequest;

    public ClimberSubsystem() {
        this.motor1 = new TalonFX(Constants.MOTOR_1_ID);
        this.motor2 = new TalonFX(Constants.MOTOR_2_ID);

        this.motor1.getConfigurator().apply(Constants.TALON_FX_CONFIGURATION);
        this.motor2.getConfigurator().apply(Constants.TALON_FX_CONFIGURATION);

        this.motor1MotionMagicVoltageRequest = new MotionMagicVoltage(0);
        this.motor2MotionMagicVoltageRequest = new MotionMagicVoltage(0);
    }

    public void moveToPosition(double rotations) {
        this.motor1.setControl(this.motor1MotionMagicVoltageRequest.withPosition(rotations));
    }

    public void stopMotor() {
        this.motor1.stopMotor();
    }

    private double degreesToEncoderUnits(double degrees) {
        final double unitsPerRevolution = 2048;
        return (degrees / 360.0) * unitsPerRevolution * Constants.GEARBOX_RATIO;
    }

    private double degreesWithGearboxRatio(double degrees) {
        return (degrees / 360.0) * Constants.GEARBOX_RATIO;
    }

    public void setPosition(double rotations) {
        this.motor1MotionMagicVoltageRequest.withPosition(rotations * Constants.GEARBOX_RATIO);
        this.motor1.setControl(motor1MotionMagicVoltageRequest);
    }

    public double getCurrentPosition() {
        return this.motor1.getPosition().getValueAsDouble() * 360.0 / Constants.GEARBOX_RATIO;
    }

    public boolean atSetpoint(double targetPosition) {
        return Math.abs(this.getCurrentPosition() - targetPosition) < 0.1;
    }

    public Command ascendCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> this.motor1
                        .setControl(
                                this.motor1MotionMagicVoltageRequest.withPosition(this.degreesWithGearboxRatio(125)))),
                Commands.runOnce(() -> this.motor2
                        .setControl(this.motor2MotionMagicVoltageRequest.withPosition(degreesWithGearboxRatio(-125)))),
                Commands.runOnce(() -> motor1.setNeutralMode(NeutralModeValue.Brake)),
                Commands.runOnce(() -> motor2.setNeutralMode(NeutralModeValue.Brake))
                
        );
    }
    
    public Command descendCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> this.motor1
                        .setControl(
                                this.motor1MotionMagicVoltageRequest
                                        .withPosition(this.degreesWithGearboxRatio(this.degreesToEncoderUnits(0))))),
                Commands.runOnce(() -> this.motor2
                        .setControl(this.motor2MotionMagicVoltageRequest.withPosition(this.degreesToEncoderUnits(0)))));
    }
}
