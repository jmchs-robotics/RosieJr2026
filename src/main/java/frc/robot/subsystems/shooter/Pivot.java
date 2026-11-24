package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {

  private final TalonFX pivotMotor1;

  public Pivot() {

    pivotMotor1 = new TalonFX(PivotConstants.pivotMotorID);
  }

  public void stopMotor() {
    pivotMotor1.stopMotor();
  }

  public void setPosition(Angle angle) {
    pivotMotor1.setControl(new PositionVoltage(angle.in(Units.Rotations)));
  }
}
