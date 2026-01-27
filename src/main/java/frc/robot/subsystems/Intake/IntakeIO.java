package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {

    public double intakeCurrentAmps = 0.0;
    public double intakeAppliedVolts = 0.0;
    // public double intakeVelocityRotPerSec = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setOpenLoop(double speed) {}
}
