package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public class IntakeIOSparkFlex implements IntakeIO {

  private final SparkFlex intakeMotor;
  private final TalonFX intakeSlapDownMotor;

  public IntakeIOSparkFlex() {

    intakeMotor = new SparkFlex(10, MotorType.kBrushless);
    intakeSlapDownMotor = new TalonFX(11);

    intakeSlapDownMotor.setPosition(0);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Rotations.of(0).in(Units.Rotations);
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

  @Override
  public void setPosition(Angle angle) {
    intakeSlapDownMotor.setControl(new PositionVoltage(angle.in(Units.Rotations)));
  }
}
