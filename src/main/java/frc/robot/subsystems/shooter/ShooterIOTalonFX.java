package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.LoggedTunableNumber;

public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX shooterMotor;
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("shooter kP");

  static {
    kP.initDefault(0.05);
  }

  public ShooterIOTalonFX() {

    shooterMotor = new TalonFX(9);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    inputs.shooterAppliedVolts = shooterMotor.getSupplyVoltage().getValueAsDouble();
    inputs.shooterCurrentAmps = shooterMotor.getSupplyCurrent().getValueAsDouble();
    inputs.shooterVelocityRotPerSec = shooterMotor.getRotorVelocity().getValueAsDouble();
    shooterMotor.getConfigurator().apply(new Slot0Configs().withKP(kP.get()));
  }

  @Override
  public void setOpenLoop(double speed) {
    // shooterMotor.set(-speed);
    shooterMotor.setControl(new VelocityVoltage(speed));
  }
}
