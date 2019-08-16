/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4499.robot.commands.presets;

import org.usfirst.frc4499.robot.commands.SliderSetPositionMM;
import org.usfirst.frc4499.robot.commands.SliderSetPositionPID;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class setSlider extends CommandGroup {
  /**
   * Add your docs here.
   */
  public setSlider(double slider) {
    addSequential(new WaitCommand(1));
    addSequential(new SliderSetPositionMM(slider));
    // Add Commands here:
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires elevator,
    // a CommandGroup containing them would require both the chassis and the
    // elevator.
  }
}
