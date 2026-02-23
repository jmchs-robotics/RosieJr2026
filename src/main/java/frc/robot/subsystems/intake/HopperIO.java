package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {

  @AutoLog
  public static class HopperIOInputs {

    public double hopperCurrentAmps = 0.0;
    public double hopperAppliedVolts = 0.0;
    public double hopperVelocityRotPerSec = 0.0;
  }

  public default void updateInputs(HopperIOInputs inputs) {}

  public default void setOpenLoop(double speed) {}
}
