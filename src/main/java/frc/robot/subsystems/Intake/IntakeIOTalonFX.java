package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;

    public class IntakeIOTalonFX implements IntakeIO {
    
        private final TalonFX intakeMotor;

        public IntakeIOTalonFX() {

            intakeMotor = new TalonFX(10);
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
