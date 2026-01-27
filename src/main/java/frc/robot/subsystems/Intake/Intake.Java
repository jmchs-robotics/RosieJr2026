package frc.robot.subsystems.intake;
//kraken 60 and neo vortex
// addie put this in to check
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {


    private final IntakeIO io;

    private final IntakeIOInputsAutoLogged inputs;

    public Intake(IntakeIO io) {
        this.io = io;
        inputs = new IntakeIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void setMotor(double speed) {
        io.setOpenLoop(speed);
    }
}