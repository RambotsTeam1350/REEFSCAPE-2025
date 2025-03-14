package frc.robot.subsystems.L4Coral;

import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ElevatorSubsystem;

public class L4Coral {
    private final WristSubsystem wristSubsystem = new WristSubsystem();
    private final ShoulderSubsystem shoulderSubsystem = new ShoulderSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    public Command L4Coral(){

        return Commands.sequence(
            Commands.runOnce(()  -> shoulderSubsystem.ShoulderToLevel4()),
            Commands.runOnce(()  -> wristSubsystem.WristToLevel4()),
            Commands.runOnce(()  -> elevatorSubsystem.L4Command())

        );
    }
}