package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class TurretIOTalonFX implements TurretIO {

  private final TalonFX turretMotor;
  private final TalonFXConfiguration config;

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
  public Rotation2d getPosition() {
    return new Rotation2d(Units.rotationsToRadians(getTurretPositionAsDouble()));
  }

  @Override
  public double getTurretPositionAsDouble() {
    return turretMotor.getPosition().getValueAsDouble();
  }
}
