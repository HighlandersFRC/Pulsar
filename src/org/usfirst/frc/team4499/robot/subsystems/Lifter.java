package org.usfirst.frc.team4499.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team4499.robot.RobotMap;
import org.usfirst.frc.team4499.robot.RobotStats;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;

import org.usfirst.frc.team4499.robot.OI;
import org.usfirst.frc.team4499.robot.commands.*;
import org.usfirst.frc.team4499.robot.Robot;



/**
 *
 */
public class Lifter extends Subsystem {
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	LifterGoToTicks lifterSet;
	ControlLifter controlLifter;
	
	CANTalon lifterMotor = RobotMap.lifterMotorMaster;
	
	public void stop() {
		//if (controlLifter.isRunning()){
		//controlLifter.cancel();
		//}
		//RobotMap.lifterMotorMaster.set(0);
	}
	
	public void controlLifter() {
		RobotMap.lifterMotorMaster.setVoltageRampRate(30);
		//System.out.println("Running controlLifter");
		//System.out.println(RobotStats.endTicks);

		lifterMotor.changeControlMode(TalonControlMode.PercentVbus);
		controlLifter = new ControlLifter();
		controlLifter.start();
	}
	
	public void lifterGoToTicks(int ticks) {
		LifterGoToTicks goToTicks = new LifterGoToTicks(ticks);
		goToTicks.start();
	}
	
    public void initDefaultCommand() {
    	
    	// Set the default command for a subsystem here.
    	
    	//Hold current position
    	//setDefaultCommand(new LifterGoToTicks(RobotMap.lifterMotorMaster.getEncPosition()));
    	//setDefaultCommand(new LifterGoToTicks(3000));
    	lifterSet = new LifterGoToTicks(RobotStats.endTicks);
    	setDefaultCommand(lifterSet);
    }
}

