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

public class ClimberSubsystem extends SubsystemBase {
    
    public final TalonFX climberMotorOne;
    private final TalonFX climberMotorTwo;
    private final double gearBoxRatio = 20;
    private final double rotationAmount = 0.375;
    public final StatusSignal<Angle> position;
    public double positionDouble = 0.0;

    private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);

    public ClimberSubsystem() {
        climberMotorOne = new TalonFX(15);
        climberMotorTwo = new TalonFX(14);

        position = climberMotorOne.getPosition();

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

        climberMotorOne.getConfigurator().apply(cfg);
        climberMotorTwo.getConfigurator().apply(cfg);
    }

    public void moveToPosition(double rotations) {
        climberMotorOne.setControl(motionMagicControl.withPosition(rotations));
    }
    
    public void stopClimberMotor() {
        climberMotorOne.setControl(new MotionMagicVoltage(0)); // Set to zero voltage
    }
    
    private double degreesToEncoderUnits(double degrees) {
        // Assuming 2048 units per revolution and a gear ratio of 9:1
        double unitsPerRevolution = 2048;
        return (degrees / 360.0) * unitsPerRevolution * gearBoxRatio;
    }
    private double degreesWithGearBoxRatio(double degrees) {
        // Assuming 2048 units per revolution and a gear ratio of 9:1
        //double unitsPerRevolution = 2048;
        return (degrees / 360.0) * gearBoxRatio;
    }
    // following commands will all be used in a single command to move the climber
    public void setPosition(double rotations) {
        motionMagicControl.Position = rotations * gearBoxRatio;
        climberMotorOne.setControl(motionMagicControl);
    }
    
    public double getCurrentPosition() {
        return climberMotorOne.getPosition().getValueAsDouble() * 360 / gearBoxRatio;
    }
    
    public boolean atSetpoint(double targetPosition) {
        return Math.abs(getCurrentPosition() - targetPosition) < 0.1;
    }
    
    // using those pieces to run the motor
        // public Command moveClimberCommand(double targetRotations) {
        //     return Commands.runOnce(() -> setPosition(targetRotations), this).until(
        //         () -> atSetpoint(targetRotations)
        //         ).andThen(() -> stopClimberMotor());
        // }
    
        public Command ascendCommand() {
            // create a Motion Magic request, voltage output
            final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
           //System.out.println();
            return Commands.sequence(
               
            // set target position to 100 rotations
                Commands.runOnce (() -> climberMotorOne.setControl(m_request.withPosition(degreesWithGearBoxRatio(135)))),
                Commands.runOnce (() -> climberMotorTwo.setControl(m_request.withPosition(degreesWithGearBoxRatio(-135))))
            );
        }
    
        public Command descendCommand() {
            // create a Motion Magic request, voltage output
            final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    
            return Commands.sequence(
                // set target position to 100 rotations
                Commands.runOnce (() -> climberMotorOne.setControl(m_request.withPosition(degreesToEncoderUnits(0)))),
                Commands.runOnce (() -> climberMotorTwo.setControl(m_request.withPosition(degreesToEncoderUnits(0))))
            );
        }

}
