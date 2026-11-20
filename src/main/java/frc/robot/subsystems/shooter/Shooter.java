package frc.robot.subsystems.shooter;

// TODO change to sparkflex
// TODO make command
// TODO implement in RobotContainer

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.ShooterConstants;

public class Shooter {

  private final SparkMax shooterRightMotor;
  private final SparkMax shooterLeftMotor;

  public Shooter() {

    shooterRightMotor = new SparkMax(ShooterConstants.rightMotorID, MotorType.kBrushless);
    shooterLeftMotor = new SparkMax(ShooterConstants.leftMotorID, MotorType.kBrushless);
  }

  public void stopWheelMotors() {
    shooterRightMotor.stopMotor();
    shooterLeftMotor.stopMotor();
  }

  public void setWheelMotors(double speed) {
    shooterRightMotor.set(speed);
    shooterLeftMotor.set(-speed);
  }
}
