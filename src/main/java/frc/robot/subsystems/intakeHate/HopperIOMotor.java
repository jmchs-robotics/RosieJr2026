package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class HopperIOMotor implements HopperIO {

  private final SparkFlex hopperMotor;

  public HopperIOMotor() {

    hopperMotor = new SparkFlex(12, MotorType.kBrushless);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {

    inputs.hopperAppliedVolts = hopperMotor.getBusVoltage() * hopperMotor.getAppliedOutput();
    inputs.hopperCurrentAmps = hopperMotor.getOutputCurrent();
    inputs.hopperVelocityRotPerSec = hopperMotor.getEncoder().getVelocity();
  }

  @Override
  public void setOpenLoop(double speed) {
    hopperMotor.set(speed);
  }
}
