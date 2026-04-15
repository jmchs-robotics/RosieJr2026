package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  @AutoLog
  public static class TurretIOInputs {

    public double turretCurrentAmps = 0;
    public double turretAppliedVolts = 0;
    public double turretVelocityRotPerSec = 0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default Rotation2d getPosition() {
    return new Rotation2d();
  }

  public default void setPosition(Angle position) {}

  public default void setOpenLoop(double speed) {}
}
