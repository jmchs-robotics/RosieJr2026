package frc.robot.subsystems.shooter;

// TODO implement in RobotContainer

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  private final SparkFlex shooterRightMotor;
  private final SparkFlex shooterLeftMotor;

  public Shooter() {

    shooterRightMotor = new SparkFlex(ShooterConstants.rightMotorID, MotorType.kBrushless);
    shooterLeftMotor = new SparkFlex(ShooterConstants.leftMotorID, MotorType.kBrushless);
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
