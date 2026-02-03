package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOMotors implements IntakeIO {

  private final TalonFX intakeSlapDownMotor;

  private final SparkFlex intakeMotor;

  public IntakeIOMotors() {

    intakeSlapDownMotor = new TalonFX(11);

    intakeSlapDownMotor.setPosition(0);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.Rotations.of(0).in(Units.Rotations);

    intakeMotor = new SparkFlex(10, MotorType.kBrushless);

    config = new TalonFXConfiguration();

    config.Slot0.kP = IntakeConstants.kP;
    config.Slot0.kI = IntakeConstants.kI;
    config.Slot0.kD = IntakeConstants.kD;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = 40;

    intakeSlapDownMotor.getConfigurator().apply(config);

    intakeSlapDownMotor.setPosition(0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    // "wok" - max walley 2026
    inputs.intakeAppliedVolts = intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
    inputs.intakeCurrentAmps = intakeMotor.getOutputCurrent();
    inputs.intakeVelocityRotPerSec = intakeMotor.getEncoder().getVelocity();
    inputs.intakeSlapDownAppliedVolts = intakeSlapDownMotor.getSupplyVoltage().getValueAsDouble();
    inputs.intakeSlapDownCurrentAmps = intakeSlapDownMotor.getSupplyCurrent().getValueAsDouble();
    inputs.intakeSlapDownVelocityRotPerSec =
        intakeSlapDownMotor.getRotorVelocity().getValueAsDouble();
  }

  @Override
  public void setOpenLoop(double speed) {
    intakeSlapDownMotor.set(speed);
  }

  @Override
  public void setPosition(Angle angle) {
    intakeSlapDownMotor.setControl(new PositionVoltage(angle.in(Units.Rotations)));
  }
}
