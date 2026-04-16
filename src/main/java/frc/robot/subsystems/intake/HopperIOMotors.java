package frc.robot.subsystems.intake;

// import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.hardware.TalonFX;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import java.util.function.DoubleSupplier;

public class HopperIOMotors implements HopperIO {

  private final TalonFX hopperMotor;

  private final TalonFX shootIndexMotor;

  public HopperIOMotors() {

    hopperMotor = new TalonFX(12);

    shootIndexMotor = new TalonFX(15);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {

    inputs.ShootIndexAppliedVolts = shootIndexMotor.getSupplyVoltage().getValueAsDouble();
    inputs.shootIndexCurrentAmps = shootIndexMotor.getSupplyCurrent().getValueAsDouble();
    inputs.shootIndexVelocityRotPerSec = shootIndexMotor.getRotorVelocity().getValueAsDouble();
    inputs.shootIndexIsConnected = shootIndexMotor.isConnected();

    inputs.hopperAppliedVolts = shootIndexMotor.getSupplyVoltage().getValueAsDouble();
    inputs.hopperCurrentAmps = shootIndexMotor.getSupplyCurrent().getValueAsDouble();
    inputs.hopperVelocityRotPerSec = shootIndexMotor.getRotorVelocity().getValueAsDouble();
    inputs.hopperIsConnected = shootIndexMotor.isConnected();
  }

  @Override
  public void setOpenLoop(double speed) {
    hopperMotor.set(speed);
    shootIndexMotor.set(-speed);
  }
}
