package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import javax.lang.model.util.ElementScanner14;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class TurretIOTalonFX implements TurretIO {

  private final TalonFX turretMotor;
  private final TalonFXConfiguration config;

  private final DutyCycleEncoder throughBoreA = new DutyCycleEncoder(0);
  private final double throughBoreAOffset = 0.464;

  private final DutyCycleEncoder throughBoreB = new DutyCycleEncoder(1);
  private final double throughBoreBOffset = 0.980;

  public TurretIOTalonFX() {

    turretMotor = new TalonFX(14);
    config = new TalonFXConfiguration();

    config.Slot0.kP = 0.1;
    config.Slot0.kD = 0;

    turretMotor.getConfigurator().apply(config);

    calibrateTurret();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {

    inputs.turretAppliedVolts = turretMotor.getSupplyVoltage().getValueAsDouble();
    inputs.turretCurrentAmps = turretMotor.getSupplyCurrent().getValueAsDouble();
    inputs.turretVelocityRotPerSec = turretMotor.getRotorVelocity().getValueAsDouble();

    Logger.recordOutput("turret/throughBoreARaw", throughBoreA.get());
    Logger.recordOutput("turret/throughBoreBRaw", throughBoreB.get());
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
  public void setTurretPosition(Angle position) {
    turretMotor.setControl(new PositionVoltage(position.in(Units.Rotations)));
  }

  @AutoLogOutput(key = "turret/CRTDegrees")
  private double CRTDegrees() {
    //Define teeth counts for each gear.
    double gearATeeth = 17;
    double gearBTeeth = 13;
    double mainGearTeeth = 80;
    //Define CRT multipliers.
    //If different numbers of teeth are used for gears than 13 and 17, the crtMult will be different.
    //Theoretically, you can use modInverse to define these automatically without using brute force math.
    double crtMult17T = 170;
    double crtMult13T = 52;
    
    //Calculate numbers used for conversion of later results.
    double crtRange = gearATeeth * gearBTeeth;
    double degFactor = 360/mainGearTeeth;

    //Handle encoder offsets, including "wrap-around" to account for any negative numbers caused by subtracting the offsets.
    double throughBoreAValue = throughBoreA.get() - throughBoreAOffset;
    if (throughBoreAValue < 0) {
      throughBoreAValue = 1 + throughBoreAValue;
    }
    double throughBoreBValue = throughBoreB.get() - throughBoreBOffset;
    if (throughBoreBValue < 0) {
      throughBoreBValue = 1 + throughBoreBValue;
    }
    Logger.recordOutput("turret/throughBoreAwithOffset", throughBoreAValue);
    Logger.recordOutput("turret/throughBoreBWithOffset", throughBoreBValue);

    //Convert encoders values to representative tooth positon.
    double toothPosA = throughBoreAValue * gearATeeth;
    double toothPosB = throughBoreBValue * gearBTeeth;

    //CRT Tooth Position on a gear with teeth equal to n = (gearATeeth * gearBTeeth).
    double crtToothNumber = ((crtMult17T * throughBoreBValue) + (crtMult13T * throughBoreAValue)) % 221;
    Logger.recordOutput("turret/virtualTurretPosition", crtToothNumber);

    //Map the virtual n tooth gear to the main turret gear.
    double turretToothNumber = crtToothNumber % 80;

    //Center the desired turret range around 0, the position it shoots straight relative to front.
    //Remember units are in tooth number at this stage.
    double turretROM = 60;
    double centeredTToothNumber = turretToothNumber + (turretROM/2);
    double finalTToothNumber;

    //Reallow negative numbers to make the final result be relative to straight.
    if (centeredTToothNumber > 50) {
      finalTToothNumber = centeredTToothNumber - mainGearTeeth;
    }
    else {
      finalTToothNumber = centeredTToothNumber;
    }

    //Convert the turret's centered tooth number to representative degree of rotation.
    double turretDeg = finalTToothNumber * degFactor;
    
  }

  // @AutoLogOutput(key = "turret/calcCRT")
  // private int calculateCRT(int toothA, int toothB) {

  //   int inverse = modInverse(13, 17);

  //   int x = (toothA + 13 * (Math.floorMod((toothB - toothA) * inverse, 17))) % 221;

  //   Logger.recordOutput("turret/calcCRT", x);
  //   return x == 0 ? 221 : x;
  // }

  // @AutoLogOutput(key = "turret/modInverse")
  private int modInverse(int a, int b) {
    int currentBIncrement = b;
    int currentModInverse = 0;
    int finalModInverse = 1;

    if (b == 1) {
      return 0;
    }
    while (a > 1) {

      int q = a / b;
      int t = b;
      b = a % b;
      a = t;
      t = currentModInverse;
      currentModInverse = finalModInverse - q * currentModInverse;
      finalModInverse = t;
    }

    Logger.recordOutput("turret/modInverse", finalModInverse);
    return finalModInverse < 0 ? finalModInverse + currentBIncrement : finalModInverse;
  }

  private void calibrateTurret() {
    turretMotor.setPosition((CRTDegrees() * (200 / 7)) / 360.0);
  }
}
