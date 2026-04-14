package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShootIO {

  @AutoLog
  public static class ShootIOInputs {

    public double shootCurrentAmps = 0.0;
    public double shootAppliedVolts = 0.0;
    public double shootVelocityRotPerSec = 0.0;
    public boolean shootIsConnected = false;
  }

  public default void updateInputs(ShootIOInputs inputs) {}

  public default void setVelocity(double speed) {}
}
