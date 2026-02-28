package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

  @AutoLog
  public static class ClimbIOInputs {

    public double climbCurrentAmps = 0.0;
    public double climbAppliedvolts = 0.0;
    public double climbVelocityRotPerSec = 0.0;
  }

  public default void updateInputs(ClimbIOInputs inputs) {}

  public default void setOpenLoop(double speed) {}
}
