// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;

/** Container for auto command factories. */
public final class Autos {

        /**
         * A simple auto routine that drives backward a specified distance, and then
         * stops.
         */
        public static Command driveBack(Drivetrain drive) {
                return Commands.sequence(
                                // Drive onto the charging station
                                drive.positionDriveCommand(-130, -130)
                                                .withTimeout(5)
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders)),

                                drive.positionDriveCommand(-32, 32)
                                                .withTimeout(3)
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders)));
        }

        /**
         * A complex auto routine that drives backward, then balances
         */
        public static Command balance(Drivetrain drive) {
                return Commands.sequence(
                                // Drive onto the charging station
                                drive.positionDriveCommand(94, 94)
                                                .withTimeout(6),

                                // Balance the robot
                                drive.balance())
                                .beforeStarting(Commands.runOnce(drive::resetEncoders));
        }

        /**
         * A complex auto routine that places a game piece, picks up another one then
         * places it.
         */
        public static Command scoreTwoPieces(Drivetrain drive) {
                return Commands.sequence(

                                // Drive onto the charging station
                                drive.positionDriveCommand(-120, -120)
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders))
                                                .withTimeout(6),

                                drive.positionDriveCommand(-33, 33)
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders))
                                                .withTimeout(4),

                                drive.positionDriveCommand(20, 20)
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders))
                                                .withTimeout(4),

                                drive.positionDriveCommand(31, -31)
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders))
                                                .withTimeout(3),

                                drive.positionDriveCommand(90, 90)
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders))
                                                .withTimeout(4),

                                Autos.automatedScoringCommand(drive));
        }


        /**
         * A highly sophisticated auto routine that places a gamepiece in the hybrid
         * zone,
         * drives back, and then drives to the charging station and balances.
         */
        public static Command scoreMidAndBalance(Drivetrain drive) {
                return Commands.sequence(
                                // Drive onto the charging station
                                drive.positionDriveCommand(-84, -84)
                                                .withTimeout(6),

                                // Balance the robot
                                drive.balance())
                                .beforeStarting(Commands.runOnce(drive::resetEncoders));
        }

        /**
         * A highly sophisticated auto routine that places a gamepiece in the hybrid
         * zone,
         * drives back, and then drives to the charging station and balances.
         */
        public static Command scoreHighAndBalance(Drivetrain drive) {
                return Commands.sequence(
                                drive.positionDriveCommand(-81, -81)
                                                .withTimeout(5)
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders)),

                                drive.balance());
        }

        /**
         * A highly sophisticated auto routine that places a gamepiece in the hybrid
         * zone,
         * drives back, and then drives to the charging station and balances.
         */
        public static Command mobilityAndBalance(Drivetrain drive) {
                return Commands.sequence(
                                // Drive onto the charging station
                                drive.positionDriveCommand(-160, -160)
                                                .withTimeout(7),

                                // Drive to gyro heading
                                drive.turnToGyro(0)
                                                .withTimeout(2),

                                // Drive onto the charging station
                                drive.positionDriveCommand(90, 90)
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders))
                                                .withTimeout(2),

                                // Balance the robot
                                drive.balance())
                                .beforeStarting(Commands.runOnce(drive::resetEncoders))
                                .beforeStarting(Commands.runOnce(drive::resetGyro));
        }

        /**
         * A highly sophisticated auto routine that places a cone on the middle peg
         * autonomously, this is for driver use during teleop.
         */
        public static Command automatedScoringCommand(Drivetrain drive) {
                return Commands.sequence(
                                drive.seekTarget().withTimeout(0.75),

                                drive.goToTarget(),

                                drive.seekTarget().withTimeout(0.75));

        }

        /**
         * An incomrehnsibly sophisticated auto routine that places a cube on the tope
         * platform, drives back onto and beyond the charging station, and then drives
         * back to charging station and balances.
         */
        public static Command superAuto(Drivetrain drive) {
                return Commands.sequence(
                                                        // Drive onto the charging station
                                drive.positionDriveCommand(-170, -170)
                                                .withTimeout(7),

                                // Drive to gyro heading
                                drive.turnToGyro(0)
                                                .withTimeout(2),

                                // Drive onto the charging station
                                drive.positionDriveCommand(90, 90)
                                                .beforeStarting(Commands.runOnce(drive::resetEncoders))
                                                .withTimeout(2),

                                // Balance the robot
                                drive.balance())
                                .beforeStarting(Commands.runOnce(drive::resetEncoders))
                                .beforeStarting(Commands.runOnce(drive::resetGyro));
        }

        private Autos() {
                throw new UnsupportedOperationException("This is a utility class!");
        }
}