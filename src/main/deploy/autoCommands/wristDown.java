package frc.robot.commands.elevatorDown;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.constant.Constant;


public class wristDown {

    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final WristSubsystem wristSubsystem = new WristSubsystem();
    private final ShoulderSubsystem shoulderSubsystem = new ShoulderSubsystem();

public Command elevatorDown() {


    return Commands.sequence(
      
    Commands.runOnce(() ->  elevatorSubsystem.RestPositionCommand()),
    Commands.runOnce(() ->  wristSubsystem.coralStationCommand()),
    Commands.runOnce(() ->  shoulderSubsystem.ShoulderToCoralStation())

    );
}
}
