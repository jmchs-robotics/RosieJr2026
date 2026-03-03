package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDS extends SubsystemBase {

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);

  public LEDS(AddressableLED leds) {

    m_led = leds;

    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public AddressableLED getLeds() {
    return m_led;
  }

  public void setLEDPattern(LEDPattern pattern) {

    pattern.applyTo(m_ledBuffer);
    m_led.setData(m_ledBuffer);
  }

  public void setScrollingRainbow() {
    LEDPattern rainbow =
        LEDPattern.rainbow(255, 128)
            .scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meters.of(1 / 60));

    setLEDPattern(rainbow);
  }
}
