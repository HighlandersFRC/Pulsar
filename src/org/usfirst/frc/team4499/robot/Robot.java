
package org.usfirst.frc.team4499.robot;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import org.usfirst.frc.team4499.robot.commands.ExampleCommand;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import org.usfirst.frc.team4499.robot.commands.*;
import org.usfirst.frc.team4499.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	public static OI oi;
	
	public static Lifter lifterSubsystem;

	//LifterGoToTicks lifterSet = new LifterGoToTicks(4000);
    Command autonomousCommand;
    SendableChooser chooser;
    ConfigureTalons configTalons = new ConfigureTalons();
    

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	lifterSubsystem = new Lifter();
		oi = new OI();
        chooser = new SendableChooser();
        chooser.addDefault("Default Auto", new ExampleCommand());
        
        //Lifter = new Lifter();
        
        //Configure Talons
        
        RobotMap.motorRightOne.setInverted(true);
    	RobotMap.motorRightTwo.setInverted(true);
        
    	RobotMap.lifterMotorMaster.enableBrakeMode(true);
    	RobotMap.lifterMotorMaster.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
    	RobotMap.lifterMotorMaster.reverseSensor(true);
    	RobotMap.lifterMotorMaster.setEncPosition(0);
    	RobotMap.lifterMotorSlave.changeControlMode(TalonControlMode.Follower);
    	RobotMap.lifterMotorMaster.changeControlMode(TalonControlMode.PercentVbus);
    	RobotMap.lifterMotorMaster.setPID(0.4, 0.0006, 30, 0, 1000, 0, 0);
    	RobotMap.lifterMotorMaster.setAllowableClosedLoopErr(100);
    	RobotMap.lifterMotorMaster.setVoltageRampRate(6);
    	
    	RobotMap.lifterMotorMaster.setForwardSoftLimit(7850);
    	RobotMap.lifterMotorMaster.enableForwardSoftLimit(true);
        
        //System.out.println("Configuring Talons Test");
        //configTalons.start();
        //System.out.println("Configured Talons");
        
        
//        chooser.addObject("My Auto", new MyAutoCommand());
        SmartDashboard.putData("Auto mode", chooser);
    }
	
	/**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
     */
    public void disabledInit(){

    }
	
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString code to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the chooser code above (like the commented example)
	 * or additional comparisons to the switch structure below with additional strings & commands.
	 */
    public void autonomousInit() {
        autonomousCommand = (Command) chooser.getSelected();
        
		/* String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
		switch(autoSelected) {
		case "My Auto":
			autonomousCommand = new MyAutoCommand();
			break;
		case "Default Auto":
		default:
			autonomousCommand = new ExampleCommand();
			break;
		} */
    	
    	// schedule the autonomous command (example)
        if (autonomousCommand != null) autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    
    public void teleopInit() {
		// This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to 
        // continue until interrupted by another command, remove
        // this line or comment it out.
    	
    	//lifterSet.start();
//<<<<<<< HEAD
    	
    	
    	RobotMap.lifterMotorMaster.setVoltageRampRate(100);
    	
    	//RobotMap.lifterMotorMaster.setVoltageRampRate(16);
//=======
    
//>>>>>>> origin/Dev
    	
    	
    	
    	
    	
        if (autonomousCommand != null) autonomousCommand.cancel();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	
    	//RobotMap.robotDrive.mecanumDrive_Cartesian(OI.controllerOne.getRawAxis(4), OI.controllerOne.getRawAxis(5),0,0);
       
    	
    	System.out.println(-OI.controllerOne.getRawAxis(5));
    	
    	if (Math.abs(-OI.controllerOne.getRawAxis(5)) > .1) {
    		lifterSubsystem.controlLifter();
    	} else {
    		lifterSubsystem.stop();
    	}
    	
    	
    	//System.out.println(RobotMap.lifterMotorMaster.getEncPosition());
    	//System.out.println(RobotMap.lifterMotorMaster.getOutputVoltage());
    	//RobotMap.lifterMotorMaster.set(-OI.controllerOne.getRawAxis(5)); //Negative because the controller has up -> negative return
    	RobotMap.lifterMotorSlave.set(RobotMap.lifterMotorMaster.getDeviceID());
        
        //SmartDashboard.putNumber("Pressure", RobotMap.lifterMotorMaster.getEncPosition());
    	
    	
    	
    	
    	
    	
      // RobotMap.robotDrive.setInvertedMotor(RobotMap.robotDrive.1, 1);
        Scheduler.getInstance().run();
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
    }
}
