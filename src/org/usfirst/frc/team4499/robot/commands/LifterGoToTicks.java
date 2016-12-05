package org.usfirst.frc.team4499.robot.commands;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4499.robot.*;

/**
 *
 */
public class LifterGoToTicks extends Command {

	int ticks;
	
	
    public LifterGoToTicks(int ticks) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.ticks = ticks;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	RobotMap.lifterMotorMaster.changeControlMode(TalonControlMode.Position);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	RobotMap.lifterMotorMaster.set(ticks);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        //return ( (ticks + 50) < Math.abs(RobotMap.lifterMotorMaster.getPosition()) ) || ( (ticks - 50) < Math.abs(RobotMap.lifterMotorMaster.getPosition()) );
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	RobotMap.lifterMotorMaster.changeControlMode(TalonControlMode.PercentVbus);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
