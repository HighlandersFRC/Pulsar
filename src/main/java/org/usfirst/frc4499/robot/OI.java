


package org.usfirst.frc4499.robot;

import buttons.XBoxTriggerButton;
import org.usfirst.frc4499.controller.IHandController;
import org.usfirst.frc4499.controller.XboxController;
import org.usfirst.frc4499.robot.commands.*;
import org.usfirst.frc4499.robot.commands.presets.StowElevator;
import org.usfirst.frc4499.robot.constants.LEDPatterns;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.*;
import org.usfirst.frc4499.robot.subsystems.*;
import org.usfirst.frc4499.robot.subsystems.Elevator.ElevatorControlMode;
import org.usfirst.frc4499.robot.subsystems.Elevator.PlaceMode;
import org.usfirst.frc4499.robot.subsystems.Slider.SliderControlMode;
import org.usfirst.frc4499.utility.MPSoftwarePIDController.MPSoftwareTurnType;
import org.usfirst.frc4499.utility.MPSoftwarePIDController.MPSoftwareTurnType;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.InternalButton;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;



public class OI
{
		private static OI instance;

		private XboxController m_driverXbox;
		private XboxController m_operatorXbox;

		private OI()
		{
		  try
		  {
			// Driver controllers
			m_driverXbox = new XboxController(RobotMap.DRIVER_JOYSTICK_1_USB_ID);
			m_operatorXbox = new XboxController(RobotMap.OPERATOR_JOYSTICK_1_USB_ID);



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//			operator controller



	        XBoxTriggerButton CarriageIntake = new XBoxTriggerButton(m_operatorXbox, XBoxTriggerButton.LEFT_TRIGGER);
	        CarriageIntake.whenPressed(new SetIntakeSpeed(ToteIntake.TOTE_INTAKE_SPEED));
	        CarriageIntake.whenReleased(new SetIntakeSpeed(0.0));

	        XBoxTriggerButton CarriageEject = new XBoxTriggerButton(m_operatorXbox, XBoxTriggerButton.RIGHT_TRIGGER);
	        CarriageEject.whenPressed(new SetIntakeSpeed(ToteIntake.TOTE_EXTAKE_SPEED));
			CarriageEject.whenReleased(new SetIntakeSpeed(0.0));

			
			JoystickButton Height1 = new JoystickButton(m_operatorXbox.getJoyStick(),XboxController.RIGHT_JOYSTICK_BUTTON);
			Height1.whenPressed(new ElevatorToHeight1());

			JoystickButton lowHeight = new JoystickButton(m_operatorXbox.getJoyStick(),XboxController.LEFT_JOYSTICK_BUTTON);
			lowHeight.whenPressed(new GrabFromLoadingSatation());

			//JoystickButton stow = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.B_BUTTON);
			//stow.whenPressed(new StowElevator());
			//stow.whenPressed(new setLEDPattern(LEDPatterns.SOLID_GREEN));


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//			Driver Xbox Controler


			JoystickButton climbUp = new JoystickButton(m_driverXbox.getJoyStick(), XboxController.RIGHT_TRIGGER_AXIS);
			
			JoystickButton climbDown = new JoystickButton(m_driverXbox.getJoyStick(), XboxController.LEFT_TRIGGER_AXIS);

			JoystickButton help = new JoystickButton(m_driverXbox.getJoyStick(), XboxController.X_BUTTON);
			help.whenPressed(new ElevatorSetMode(ElevatorControlMode.JOYSTICK_MANUAL));
			help.whenReleased(new ElevatorSetMode(ElevatorControlMode.MOTION_MAGIC));
			help.whenPressed(new SliderSetMode(SliderControlMode.JOYSTICK_MANUAL));
			help.whenReleased(new SliderSetMode(SliderControlMode.JOYSTICK_PID));

			JoystickButton panic = new JoystickButton(m_driverXbox.getJoyStick(), XboxController.START_BUTTON);
			panic.whenPressed(new ResetElevatorEncoder());
			panic.whenPressed(new ResetSliderEncoder());


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//			OI buttons


			SmartDashboard.putData("switch to manuel", new SetManual());
			SmartDashboard.putData("SliderTest", new sliderTest());
			SmartDashboard.putData("elevator test", new ElevatorTest());
			//SmartDashboard.putData("elevator test2", new ElevatorTest2());

			

		  } catch (Exception e) {
			  System.err.println("An error occurred in the OI constructor");
		  }
		}

		public static OI getInstance() {
			if(instance == null) {
				instance = new OI();
			}
			return instance;
		}

		public IHandController getDriverController() {
			return m_driverXbox;
		}

		public IHandController getOperatorController()
		{
			return m_operatorXbox;
		}

		public Joystick getOperatorJoystick()
		{
			return m_operatorXbox.getJoyStick();
		}
	}
