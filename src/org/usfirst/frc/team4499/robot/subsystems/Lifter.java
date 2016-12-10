package org.usfirst.frc.team4499.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team4499.robot.RobotMap;
import edu.wpi.first.wpilibj.*;
import org.usfirst.frc.team4499.robot.OI;
import org.usfirst.frc.team4499.robot.commands.*;



/**
 *
 */
public class Lifter extends Subsystem {
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	//LifterGoToTicks lifterSet;
	
	public void stop() {
		RobotMap.lifterMotorMaster.set(0);
	}
	
	public void controlLifter() {
		RobotMap.lifterMotorMaster.set(-OI.controllerOne.getRawAxis(5));
	}
	
	public void lifterGoToTicks(int ticks) {
		LifterGoToTicks goToTicks = new LifterGoToTicks(ticks);
		goToTicks.start();
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
     //  setDefaultCommand(new LifterGoToTicks(RobotMap.lifterMotorMaster.getEncPosition()));
    	
    	//Hold current position
    	//lifterGoToTicks(talon.getEncPosition());
    	//lifterSet = new LifterGoToTicks(RobotMap.lifterMotorMaster.getEncPosition());
    	//lifterSet.start();
    }
}

