package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {

    public double intakeCurrentAmps = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeVelocityRotPerSec = 0.0;
    public double intakeSlapDownCurrentAmps = 0.0;
    public double intakeSlapDownAppliedVolts = 0.0;
    public double intakeSlapDownVelocityRotPerSec = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setOpenLoopWheels(double speed) {}

  public default void setOpenLoopSlapDown(double speed) {}

  public default void setPosition(Angle angle) {}
}
