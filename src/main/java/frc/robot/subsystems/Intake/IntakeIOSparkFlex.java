package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkFlex;

    public class IntakeIOSparkFlex implements IntakeIO {
    
        private final SparkFlex intakeMotor;

        public IntakeIOSparkFlex() {

            intakeMotor = new SparkFlex(10, null);
        }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeAppliedVolts = intakeMotor.getSupplyVoltage().getValueAsDouble();
        inputs.intakeCurrentAmps = intakeMotor.getSupplyCurrent().getValueAsDouble();
        inputs.intakeVelocityRotPerSec = intakeMotor.getRotorPosition().getValueAsDouble();
    }

    @Override
    public void setOpenLoop(double speed) {
        intakeMotor.set(speed);
    }
}
