package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {

    public double shooterCurrentAmps = 0.0;
    public double shooterAppliedVolts = 0.0;
    public double shooterVelocityRotPerSec = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setOpenLoop(double speed) {}
}
