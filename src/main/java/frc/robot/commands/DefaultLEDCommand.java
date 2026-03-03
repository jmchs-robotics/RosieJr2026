package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDS;

public class DefaultLEDCommand extends Command {

  private final LEDS m_led;

  private Alliance alliance = null;

  public DefaultLEDCommand(LEDS led) {

    m_led = led;
    addRequirements(m_led);
  }

  @Override
  public void initialize() {
    // if (DriverStation.getAlliance().isPresent()) {
    //     if (DriverStation.getAlliance().get() == Alliance.Blue) {
    //         alliance = Alliance.Blue;
    //     } else {
    //         alliance = Alliance.Red;
    //     }
    // }
    LEDPattern pattern = LEDPattern.solid(Color.kRed);
    m_led.setLEDPattern(pattern);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
