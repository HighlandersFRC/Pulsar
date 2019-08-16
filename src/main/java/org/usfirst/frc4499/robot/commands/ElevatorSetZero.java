package org.usfirst.frc4499.robot.commands;

import org.usfirst.frc4499.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ElevatorSetZero extends Command
{
	private double position;
	
	public ElevatorSetZero(double position) {
		this.position = position;
		requires(Robot.elevator);
	}

	@Override
	protected void initialize() {
		Robot.elevator.resetZeroPosition(position);
	}

	@Override
	protected void execute() {
		
	}

	@Override
	protected boolean isFinished() {
		return true;
	}

	@Override
	protected void end() {
		
	}

	@Override
	protected void interrupted() {
			
	}
}