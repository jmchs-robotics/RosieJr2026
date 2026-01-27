package frc.robot.subsystems.intake;

// import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeIOSparkFlex implements IntakeIO {

  private final SparkFlex intakeMotor;

  public IntakeIOSparkFlex() {

    intakeMotor = new SparkFlex(10, MotorType.kBrushless);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // im not sure that the getBusVoltage is right it was supplyVoltage before but thats for TalonFX
    inputs.intakeAppliedVolts = intakeMotor.getBusVoltage();
    inputs.intakeCurrentAmps = intakeMotor.getOutputCurrent();
    // inputs.intakeVelocityRotPerSec = intakeMotor.getRotorPosition().getValueAsDouble();
  }

  @Override
  public void setOpenLoop(double speed) {
    intakeMotor.set(speed);
  }
}
