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

public class ElevatorSubsystem extends SubsystemBase {
    public static final class Constants {
        public static final int MOTOR_1_ID = 16;
        public static final int MOTOR_2_ID = 17;
        public static final double GEARBOX_RATIO = 12;

        public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs().withKP(4.8).withKI(0).withKD(0.1)
                .withKV(0.12).withKA(0.01).withKS(0.25);
        public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(80).withMotionMagicAcceleration(160).withMotionMagicJerk(1600);
        public static final TalonFXConfiguration TALON_FX_CONFIGURATION = new TalonFXConfiguration()
                .withSlot0(SLOT0_CONFIGS).withMotionMagic(MOTION_MAGIC_CONFIGS);
    }

    private final TalonFX motor1;
    private final TalonFX motor2;

    private StatusSignal<Angle> motor1Position;
    private StatusSignal<Angle> motor2Position;

    private final MotionMagicVoltage motor1MotionMagicVoltageRequest;
    private final MotionMagicVoltage motor2MotionMagicVoltageRequest;

    public ElevatorSubsystem() {
        this.motor1 = new TalonFX(Constants.MOTOR_1_ID);
        this.motor2 = new TalonFX(Constants.MOTOR_2_ID);

        this.motor1Position = this.motor1.getPosition();
        this.motor2Position = this.motor2.getPosition();

        this.motor1.getConfigurator().apply(Constants.TALON_FX_CONFIGURATION);
        this.motor2.getConfigurator().apply(Constants.TALON_FX_CONFIGURATION);

        this.motor1MotionMagicVoltageRequest = new MotionMagicVoltage(0);
        this.motor2MotionMagicVoltageRequest = new MotionMagicVoltage(0);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(this.motor1Position, this.motor2Position);
        //System.out.println(motor1Position.getValueAsDouble() + " elevator motor 1");
        //System.out.println(motor2Position.getValueAsDouble() + " elevator motor 2");
    }

    private double degreesWithGearboxRatio(double degrees) {
        return (degrees / 360.0) * Constants.GEARBOX_RATIO;
    }

    public Command restPositionCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> this.motor1
                        .setControl(
                                this.motor1MotionMagicVoltageRequest.withPosition(0))),
                Commands.runOnce(() -> this.motor2
                        .setControl(this.motor2MotionMagicVoltageRequest.withPosition(0))));
    }

    public Command l1Command() {
        return Commands.sequence(
                Commands.runOnce(() -> this.motor1
                        .setControl(
                                this.motor1MotionMagicVoltageRequest.withPosition(0))),
                Commands.runOnce(() -> this.motor2
                        .setControl(this.motor2MotionMagicVoltageRequest.withPosition(0))));
    }

    public Command l2Command() {
        return Commands.sequence(
                Commands.runOnce(() -> this.motor1
                        .setControl(
                                this.motor1MotionMagicVoltageRequest.withPosition(-17.5))),
                Commands.runOnce(() -> this.motor2
                        .setControl(this.motor2MotionMagicVoltageRequest.withPosition(17.79))));
    }

    public Command l3Command() {
        return Commands.sequence(
                Commands.runOnce(() -> this.motor1
                        .setControl(
                                this.motor1MotionMagicVoltageRequest.withPosition(-45.3))),
                Commands.runOnce(() -> this.motor2
                        .setControl(this.motor2MotionMagicVoltageRequest.withPosition(45.98))));
    }

    public Command l4Command() {
        return Commands.sequence(
                Commands.runOnce(() -> this.motor1
                        .setControl(
                                this.motor1MotionMagicVoltageRequest.withPosition(40.87))),
                Commands.runOnce(() -> this.motor2
                        .setControl(this.motor2MotionMagicVoltageRequest.withPosition(-40.37))));
    }

    public Command lBargeCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> this.motor1
                        .setControl(
                                this.motor1MotionMagicVoltageRequest.withPosition(44))),
                Commands.runOnce(() -> this.motor2
                        .setControl(this.motor2MotionMagicVoltageRequest.withPosition(-44))));
    }
}
