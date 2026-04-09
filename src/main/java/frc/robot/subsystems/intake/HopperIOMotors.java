package frc.robot.subsystems.intake;

import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import java.util.function.DoubleSupplier;

public class HopperIOMotors implements HopperIO {

  private final SparkMax hopperMotor;
  private final RelativeEncoder hopperEncoder;

  private final TalonFX shootIndexMotor;

  public HopperIOMotors() {

    hopperMotor = new SparkMax(12, MotorType.kBrushless);
    hopperEncoder = hopperMotor.getEncoder();

    shootIndexMotor = new TalonFX(15);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {

    inputs.ShootIndexAppliedVolts = shootIndexMotor.getSupplyVoltage().getValueAsDouble();
    inputs.shootIndexCurrentAmps = shootIndexMotor.getSupplyCurrent().getValueAsDouble();
    inputs.shootIndexVelocityRotPerSec = shootIndexMotor.getRotorVelocity().getValueAsDouble();
    inputs.shootIndexIsConnected = shootIndexMotor.isConnected();

    sparkStickyFault = false;

    ifOk(
        hopperMotor,
        hopperEncoder::getVelocity,
        (value) -> inputs.hopperVelocityRotPerSec = value * 60);
    ifOk(
        hopperMotor,
        new DoubleSupplier[] {hopperMotor::getAppliedOutput, hopperMotor::getBusVoltage},
        (values) -> inputs.hopperAppliedVolts = values[0] * values[1]);
    ifOk(hopperMotor, hopperMotor::getOutputCurrent, (value) -> inputs.hopperCurrentAmps = value);
    inputs.hopperIsConnected = !sparkStickyFault;
  }

  @Override
  public void setOpenLoop(double speed) {
    hopperMotor.set(speed);
    shootIndexMotor.set(-speed);
  }
}
