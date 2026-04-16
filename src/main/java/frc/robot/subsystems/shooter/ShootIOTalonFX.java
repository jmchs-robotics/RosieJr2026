package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShootIOTalonFX implements ShootIO {

  private final TalonFX shootMotor;

  public ShootIOTalonFX() {

    shootMotor = new TalonFX(9);
    shootMotor.setNeutralMode(NeutralModeValue.Coast);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 0.04;
    shootMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ShootIOInputs inputs) {
    inputs.shootAppliedVolts = shootMotor.getSupplyVoltage().getValueAsDouble();
    inputs.shootCurrentAmps = shootMotor.getSupplyCurrent().getValueAsDouble();
    inputs.shootVelocityRotPerSec = shootMotor.getRotorVelocity().getValueAsDouble();
    inputs.shootIsConnected = shootMotor.isConnected();
  }

  @Override
  public void setVelocity(double speed) {
    shootMotor.setControl(new VelocityVoltage(-speed));
  }
}
