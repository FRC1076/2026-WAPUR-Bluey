package frc.robot.subsystems.grabber;



public interface GrabberIO {

    public static class GrabberIOInputs {

        /*  The Grabber has two independatly controlled motors
            so we have variables for each */
            
        public double rightAppliedVolts;
        public double leftAppliedVolts;

        public double rightPostion;
        public double leftPostion;

        //RPS is radiens per second
        public double rightRPS;
        public double leftRPS;
        
    }

    // updates inputs for the grabber
    public abstract void updateInputs(GrabberIOInputs inputs);

    // changes voltage of motors
    public abstract void rightRunVolts(double voltage);
    public abstract void leftRunVolts(double voltage);

    // sets postion of the grabbers 
    public abstract void rightSetPostion(double radiens);
    public abstract void leftSetPostion(double radiens);

    // sets velocity of the motors
    public abstract void rightSetRPS(double radsPerSec);
    public abstract void leftSetRPS(double radsPerSec);
}
