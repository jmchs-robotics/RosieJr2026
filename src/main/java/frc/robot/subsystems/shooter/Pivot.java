package frc.robot.subsystems.shooter;

// TODO implement in RobotContainer

import com.ctre.phoenix6.hardware.TalonFX;
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

  // TODO change to setposition and have it take in a position
  public void setposition(double speed) {
    pivotMotor1.set(speed);
  }
}
