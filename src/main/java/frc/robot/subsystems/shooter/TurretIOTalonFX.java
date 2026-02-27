package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class TurretIOTalonFX implements TurretIO {

  private final TalonFX turretMotor;
  private final TalonFXConfiguration config;

  private double turretOffset = 0;

  public TurretIOTalonFX() {

    turretMotor = new TalonFX(14);
    config = new TalonFXConfiguration();

    config.Slot0.kP = 1;
    config.Slot0.kD = 0;

    turretMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {

    inputs.turretAppliedVolts = turretMotor.getSupplyVoltage().getValueAsDouble();
    inputs.turretCurrentAmps = turretMotor.getSupplyCurrent().getValueAsDouble();
    inputs.turretVelocityRotPerSec = turretMotor.getRotorVelocity().getValueAsDouble();
  }

  @Override
  public double getTurretPosition() {
    return turretMotor.getPosition().getValueAsDouble() + turretOffset;
  }
}
