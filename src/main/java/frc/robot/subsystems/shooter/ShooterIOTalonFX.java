package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX shooterMotor;
  private final TalonFX followerMotor;

  public ShooterIOTalonFX() {

    shooterMotor = new TalonFX(9);
    shooterMotor.setNeutralMode(NeutralModeValue.Coast);
    followerMotor = new TalonFX(15);
    followerMotor.setControl(new Follower(9, MotorAlignmentValue.Aligned));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterAppliedVolts = shooterMotor.getSupplyVoltage().getValueAsDouble();
    inputs.shooterCurrentAmps = shooterMotor.getSupplyCurrent().getValueAsDouble();
    inputs.shooterVelocityRotPerSec = shooterMotor.getRotorVelocity().getValueAsDouble();
  }

  @Override
  public void setVelocity(double speed) {
    shooterMotor.set(-speed);
  }
}
