package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public class HoodIOTalonFX implements HoodIO {

  private final TalonFX hoodMotor;
  private final TalonFXConfiguration config;

  public HoodIOTalonFX() {

    hoodMotor = new TalonFX(13);
    config = new TalonFXConfiguration();

    config.Slot0.kP = 1;
    config.Slot0.kD = 0;

    hoodMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {

    inputs.hoodAppliedVolts = hoodMotor.getSupplyVoltage().getValueAsDouble();
    inputs.hoodCurrentAmps = hoodMotor.getSupplyCurrent().getValueAsDouble();
    inputs.hoodVelocityRotPerSec = hoodMotor.getRotorVelocity().getValueAsDouble();
  }

  @Override
  public void setOpenLoop(double speed) {
    hoodMotor.set(speed);
  }

  @Override
  public void setHoodPosition(Angle angle) {
    hoodMotor.setControl(new PositionVoltage(angle.in(Units.Rotations)));
  }

  @Override
  public double getHoodPosition() {
    return hoodMotor.getPosition().getValueAsDouble();
  }
}
