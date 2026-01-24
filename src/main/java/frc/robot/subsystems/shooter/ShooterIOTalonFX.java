package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX shooterMotor;

  public ShooterIOTalonFX() {

    shooterMotor = new TalonFX(9);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {}

  @Override
  public void setOpenLoop(double speed) {
    shooterMotor.set(speed);
  }
}
