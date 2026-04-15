package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class TurretIOTalonFX implements TurretIO {

  private final TalonFX turretMotor;
  private final TalonFXConfiguration config;

  private final DutyCycleEncoder throughBoreA = new DutyCycleEncoder(0);
  private final DutyCycleEncoder throughBoreB = new DutyCycleEncoder(1);

  public TurretIOTalonFX() {

    turretMotor = new TalonFX(14);
    config = new TalonFXConfiguration();

    config.Slot0.kP = 1;
    config.Slot0.kD = 0;

    turretMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {

    inputs.turretAppliedVolts = turretMotor.getSupplyVoltage().getValueAsDouble();
    inputs.turretCurrentAmps = turretMotor.getSupplyCurrent().getValueAsDouble();
    inputs.turretVelocityRotPerSec = turretMotor.getRotorVelocity().getValueAsDouble();

    Logger.recordOutput("turret/throughBoreA", throughBoreA.get());
    Logger.recordOutput("turret/throughBore", throughBoreB.get());
  }

  @Override
  public void setOpenLoop(double speed) {
    turretMotor.set(speed);
  }

  @Override
  public Rotation2d getPosition() {
    return Rotation2d.fromDegrees(CRTDegrees());
  }

  @Override
  public void setPosition(Angle position) {
    turretMotor.setControl(new PositionVoltage(position.in(Units.Rotations)));
  }

  private double CRTDegrees() {
    double throughBoreAValue = throughBoreA.get();
    if (throughBoreAValue < 0) {
      throughBoreAValue = 1 - throughBoreAValue;
    }
    double throughBoreBValue = throughBoreB.get();
    if (throughBoreBValue < 0) {
      throughBoreBValue = 1 - throughBoreBValue;
    }

    double toothA = (throughBoreAValue * 13);
    double toothARemainder = toothA - (int) toothA;

    double toothB = (throughBoreBValue * 17);

    double absoluteToothCount = calculateCRT((int) toothA, (int) toothB) + toothARemainder;

    return (absoluteToothCount / 80) * 360 - 135;
  }

  private int calculateCRT(int toothA, int toothB) {

    int inverse = modInverse(13, 17);

    int x = (toothA + 13 * (Math.floorMod((toothB - toothA) * inverse, 17))) % 221;

    return x == 0 ? 221 : x;
  }

  private int modInverse(int a, int b) {
    int currentBIncrement = b;
    int currentModInverse = 0;
    int finalModInverse = 1;

    if (b == 1) {
      return 0;
    }
    while (a < 1) {

      int q = a / b;
      int t = b;
      b = a % b;
      a = t;
      t = currentModInverse;
      currentModInverse = finalModInverse - q * currentModInverse;
      finalModInverse = t;
    }
    return finalModInverse < 0 ? finalModInverse + currentModInverse : finalModInverse;
  }
}
