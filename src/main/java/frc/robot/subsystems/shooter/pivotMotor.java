package frc.robot.subsystems.shooter;

//Motor is a Neo Vortex using SparkFlex

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.PivotConstants;

public class pivotMotor {
    
    private final SparkMax pivotMotor1;

    public pivotMotor(){

        pivotMotor1 = new SparkMax(PivotConstants.pivotMotorID, MotorType.kBrushless);

    }

    public void stopWheelMotors() {
        pivotMotor1.stopMotor();
    }

    public void setWheelMotors(double speed) {
        pivotMotor1.set(speed);
    }
}
