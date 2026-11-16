package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class shooterWheels {
    
    private final SparkMax shooterRightMotor;
    private final SparkMax shooterLeftMotor;

    public shooterWheels() {

        shooterRightMotor = new SparkMax(shooterConstants.rightMotorID, MotorType.kBrushless);
        shooterLeftMotor = new SparkMax(shooterConstants.leftMotorID, MotorType.kBrushless);
        
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
