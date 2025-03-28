package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.Commands;

public class LEDCandle {
    private final CANdle CANdle = new CANdle(24, "rio");
    private final CANdleConfiguration CANdleConfiguration;
   // private CurrentState currentState = CurrentState.OFF;

  /*   public enum CurrentState {
        OFF,
        BLUE,
        ORANGE,
        GREEN,
        RAINBOW
    }*/

    public LEDCandle() {

        this.CANdleConfiguration = new CANdleConfiguration();
        this.CANdleConfiguration.statusLedOffWhenActive = false;
        this.CANdleConfiguration.disableWhenLOS = false;
        this.CANdleConfiguration.stripType = LEDStripType.RGB;
        this.CANdleConfiguration.brightnessScalar = 1;
        this.CANdleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;

        this.CANdle.configAllSettings(this.CANdleConfiguration, 100);

    }

public Command LEDRed() {
return Commands.sequence(
        Commands.runOnce(() -> this.CANdle.setLEDs(255, 0, 0))
);
}

public Command LEDGreen() {
    return Commands.sequence(
            Commands.runOnce(() -> this.CANdle.setLEDs(0, 255, 0))
    );
}

public Command LEDYellow() {
    return Commands.sequence(
            Commands.runOnce(() -> this.CANdle.setLEDs(255, 255, 0))
    );
    }

public Command LEDOff() {
    return Commands.sequence(
            Commands.runOnce(() -> this.CANdle.setLEDs(0, 0, 0))
    );
    }

    public Command LEDBlue() {
    return Commands.sequence(
            Commands.runOnce(() -> this.CANdle.setLEDs(0, 0, 255))
    );

    }
public Command LEDPurple() {
    return Commands.sequence(
        Commands.runOnce(() -> this.CANdle.setLEDs(136, 0, 212))
    );

}

public Command LEDWhite() {
    return Commands.sequence(
        Commands.runOnce(() -> this.CANdle.setLEDs(255, 255, 255))
    );
}

public Command LEDPink() {
    return Commands.sequence(
        Commands.runOnce(() -> this.CANdle.setLEDs(255, 125, 200))
    );

}

public Command LEDOrange() {
        return Commands.sequence(
            Commands.runOnce(() -> this.CANdle.setLEDs(255, 153, 0))
        );
    }

public Command LEDCyan() {

    return Commands.sequence(
        Commands.runOnce(() -> this.CANdle.setLEDs(0, 255, 255))
    );
}

public Command LEDBrown() {
    return Commands.sequence(
        Commands.runOnce(() -> this.CANdle.setLEDs(137, 81, 41))
    );
}

public Command LEDMaroon() {
    return Commands.sequence(
        Commands.runOnce(() -> this.CANdle.setLEDs(128, 0, 0))
    );
}

public Command LEDRainbow() {
    return Commands.sequence(
        Commands.runOnce(() -> this.CANdle.setLEDs(0, 0, 0)),
        Commands.runOnce(() -> this.CANdle.setLEDs(255, 0, 0)),
        Commands.runOnce(() -> this.CANdle.setLEDs(255, 255, 0)),
        Commands.runOnce(() -> this.CANdle.setLEDs(0, 255, 0)),
        Commands.runOnce(() -> this.CANdle.setLEDs(0, 255, 255)),
        Commands.runOnce(() -> this.CANdle.setLEDs(0, 0, 255)),
        Commands.runOnce(() -> this.CANdle.setLEDs(255, 0, 255))
    );


}


}
