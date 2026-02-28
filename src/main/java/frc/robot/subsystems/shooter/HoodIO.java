package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

  @AutoLog
  public static class HoodIOInputs {

    public double hoodCurrentAmps = 0.0;
    public double hoodAppliedVolts = 0.0;
    public double hoodVelocityRotPerSec = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setOpenLoop(double speed) {}

  public default void setHoodPosition(Angle angle) {}

  public default double getHoodPosition() {
    return 0;
  }
}
