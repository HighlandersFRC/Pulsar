package org.usfirst.frc4499.robot.commands;

import org.usfirst.frc4499.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SliderSetPositionMM extends Command {
	
	private double targetPositionInches;
	private boolean isAtTarget;
	private static final double MIN_DELTA_TARGET = 20;

    public SliderSetPositionMM(double targetPositionInches) {
    	this.targetPositionInches = targetPositionInches;
        requires(Robot.slider);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
      
    	if (Math.abs(targetPositionInches - Robot.slider.getPositionInches()) < MIN_DELTA_TARGET) {
    		isAtTarget = true;
    	}
    	else {
        	isAtTarget = false;
        	Robot.slider.setPositionMM(targetPositionInches);
    	}
//    	System.out.println("Elevator set MP initialized, target = " + targetPositionInches);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isAtTarget || Robot.slider.isFinished();
    }

    // Called once after isFinished returns true
    protected void end() {
		Robot.slider.setPositionMM(Robot.slider.getPositionInches());
//    	System.out.println("Elevator set MP end");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
//    	System.out.println("ElevatorSetPositionMP interrupted");
    	Robot.slider.setFinished(true);
		Robot.slider.setPositionMM(Robot.slider.getPositionInches());
    }
}
