// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOSparkMax;

public class RobotContainer {
  private final Drive drive;
  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive = new Drive(new DriveIOSparkMax());
        break;
      case SIM:
        drive = new Drive(new DriveIOSim());
        break;
      default:
        drive = new Drive(new DriveIO() {});
        break;
    }

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        Commands.run(
            () -> drive.driveArcade(-Math.pow(controller.getLeftY(), 2), Math.pow(controller.getRightX(), 2)), drive));
  }

  public Command getAutonomousCommand() {
    return Commands.run(() -> drive.driveVelocity(2, 2), drive);
  }
}
