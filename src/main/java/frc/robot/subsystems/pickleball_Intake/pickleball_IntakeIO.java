
package frc.robot.subsystems.pickleball_Intake;

interface pickleball_IntakeIO {

    public class pickleball_IntakeIOInputs {

        public double appliedVolts;
        public double postion;
        public double RPS;

    }

    public abstract void updateInputs(pickleball_IntakeIOInputs inputs);

    public abstract void runVolts(double voltage);

    public abstract void setRPS(double RadsPerSec);

    
}