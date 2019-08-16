package org.usfirst.frc4499.robot.subsystems;

import org.usfirst.frc4499.robot.RobotMap;
import org.usfirst.frc4499.robot.commands.*;
import org.usfirst.frc4499.robot.subsystems.Drive.DriveControlMode;
import org.usfirst.frc4499.utility.TalonSRXEncoder;
import org.usfirst.frc4499.utility.Looper;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc4499.robot.OI;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;



/**
 *
 */
public class ToteIntake extends Subsystem {

	private WPI_TalonSRX ToteIntakeMain;
	private WPI_TalonSRX ToteIntake2;
	public static enum ToteIntakeControlMode { JOYSTICK, MP_STRAIGHT, HOLD, MANUAL};
	public static final double TOTE_INTAKE_SPEED = -1.0;
	public static final double TOTE_EXTAKE_SPEED = 1.0;
	public static final double TOTE_STOP_SPEED = 0;
	/////^^^^^^^^^ replace this line with the modes we need
	
	
	private boolean isFinished;
	//private CarriageControlMode controlMode = CarriageControlMode.JOYSTICK;
	
	
	
	
	public ToteIntake() {
		try {
			ToteIntakeMain = new WPI_TalonSRX(RobotMap.INTAKE_WHEEL1_ID);
			ToteIntake2 = new WPI_TalonSRX(RobotMap.INTAKE_WHEEL2_ID);
			ToteIntake2.set(ControlMode.Follower, RobotMap.INTAKE_WHEEL1_ID);
			ToteIntake2.setInverted(true);
			//\][carriageLeft.set(CurrentLimit, value);
			
    }
		catch (Exception e) {
			System.err.println("An error occurred in the Tote Intake constructor");
			
			
		}
    }
	
		public void setWheelSpeed(double speed) {
			ToteIntakeMain.set(-speed);
    }


    @Override
    public void periodic() {
    	
    }
    	public void initDefaultCommand() {
    }
}
