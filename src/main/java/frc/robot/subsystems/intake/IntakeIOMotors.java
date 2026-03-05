package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.LoggedTunableNumber;

public class IntakeIOMotors implements IntakeIO {

  private final TalonFX intakeSlapDownMotor;

  private final SparkFlex intakeMotor;
  private final LoggedTunableNumber kP = new LoggedTunableNumber("intake kP");
  private final LoggedTunableNumber kD = new LoggedTunableNumber("intake kD");
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public IntakeIOMotors() {

    intakeSlapDownMotor = new TalonFX(11);

    intakeSlapDownMotor.setPosition(0);

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.Rotations.of(0).in(Units.Rotations);

    intakeMotor = new SparkFlex(10, MotorType.kBrushless);

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = 40;

    intakeSlapDownMotor.getConfigurator().apply(new Slot0Configs().withKP(kP.get()));
    intakeSlapDownMotor.getConfigurator().apply(new Slot0Configs().withKD(kD.get()));

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
    intakeSlapDownMotor.getConfigurator().apply(config);
  }

  @Override
  public void setOpenLoopSlapDown(double speed) {
    intakeSlapDownMotor.set(speed);
  }

  @Override
  public void setOpenLoopWheels(double speed) {
    intakeMotor.set(speed);
  }

  @Override
  public void setPosition(Angle angle) {
    intakeSlapDownMotor.setControl(new PositionVoltage(angle.in(Units.Rotations)));
  }

  @Override
  public double getPosition() {
    return intakeSlapDownMotor.getPosition().getValueAsDouble();
  }
}
