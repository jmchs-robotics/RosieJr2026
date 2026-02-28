package frc.robot.subsystems.climb;

import com.ctre.phoenix6.hardware.TalonFX;

public class ClimbIOTalonFX implements ClimbIO {

  private final TalonFX climbMotor;

  public ClimbIOTalonFX() {

    climbMotor = new TalonFX(13);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.climbAppliedvolts = climbMotor.getSupplyVoltage().getValueAsDouble();
    inputs.climbCurrentAmps = climbMotor.getSupplyCurrent().getValueAsDouble();
    inputs.climbVelocityRotPerSec = climbMotor.getRotorVelocity().getValueAsDouble();
  }

  @Override
  public void setOpenLoop(double speed) {
    climbMotor.set(speed);
  }
}
