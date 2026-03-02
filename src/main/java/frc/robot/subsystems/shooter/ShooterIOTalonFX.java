package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX shooterMotor;

  public ShooterIOTalonFX() {

    shooterMotor = new TalonFX(9);
    shooterMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterAppliedVolts = shooterMotor.getSupplyVoltage().getValueAsDouble();
    inputs.shooterCurrentAmps = shooterMotor.getSupplyCurrent().getValueAsDouble();
    inputs.shooterVelocityRotPerSec = shooterMotor.getRotorVelocity().getValueAsDouble();
  }

  @Override
  public void setOpenLoop(double speed) {
    shooterMotor.set(-speed);
  }
}
