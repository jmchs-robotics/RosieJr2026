package frc.robot.subsystems.shooter;

// TODO change to TalonFX
// TODO make command
// TODO implement in RobotContainer

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.PivotConstants;

public class Pivot {

  private final SparkMax pivotMotor1;

  public Pivot() {

    pivotMotor1 = new SparkMax(PivotConstants.pivotMotorID, MotorType.kBrushless);
  }

  public void stopWheelMotors() {
    pivotMotor1.stopMotor();
  }

  // TODO change to setposition and have it take in a position
  public void setWheelMotors(double speed) {
    pivotMotor1.set(speed);
  }
}
