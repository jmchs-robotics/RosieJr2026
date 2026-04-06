package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShootIO {

  @AutoLog
  public static class ShooterIOInputs {

    public double shootCurrentAmps = 0.0;
    public double shootAppliedVolts = 0.0;
    public double shootVelocityRotPerSec = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVelocity(double speed) {}
}
