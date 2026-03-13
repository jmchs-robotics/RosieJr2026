package frc.robot.subsystems.intake;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import java.util.function.DoubleSupplier;

public class HopperIOMotor implements HopperIO {

  private final SparkFlex hopperMotor;
  private final RelativeEncoder hopperEncoder;

  public HopperIOMotor() {

    hopperMotor = new SparkFlex(12, MotorType.kBrushless);
    hopperEncoder = hopperMotor.getEncoder();
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {

    // inputs.hopperAppliedVolts = hopperMotor.getBusVoltage() * hopperMotor.getAppliedOutput();
    // inputs.hopperCurrentAmps = hopperMotor.getOutputCurrent();
    // inputs.hopperVelocityRotPerSec = hopperMotor.getEncoder().getVelocity();
    // inputs.hopperIsConnected = sparkStickyFault;

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
  }
}
