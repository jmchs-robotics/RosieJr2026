package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

public class ShootIOTalonFX implements ShootIO {

  private final TalonFX shooterMotor;

  public ShootIOTalonFX() {

    shooterMotor = new TalonFX(9);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shootAppliedVolts = shooterMotor.getSupplyVoltage().getValueAsDouble();
    inputs.shootCurrentAmps = shooterMotor.getSupplyCurrent().getValueAsDouble();
    inputs.shootVelocityRotPerSec = shooterMotor.getRotorVelocity().getValueAsDouble();
  }

  @Override
  public void setOpenLoop(double speed) {
    shooterMotor.set(-speed);
  }
}
