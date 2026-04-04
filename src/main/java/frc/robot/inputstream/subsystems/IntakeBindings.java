//package frc.robot.inputstream.subsystems;
//
//import edu.wpi.first.units.measure.Time;
//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.robot.inputstream.InputStream;
//import lombok.Setter;
//
//import java.util.Objects;
//import java.util.function.Supplier;
//
//import static edu.wpi.first.units.Units.Seconds;
//import static frc.robot.RobotContainer.subsystems;
//
///**
// * Intake button bindings.
// */
//public class IntakeBindings {
//    /**
//     * Checks if this subsystem is present, if not, don't bind anything.
//     */
//    private final boolean isPresent;
//    /**
//     * The default duty cycle speed to run at.
//     */
//    @Setter
//    private double intakeSpeed = 1;
//
//    /**
//     * The {@link InputStream} to attach to.
//     */
//    private final InputStream inputStream;
//
//    private IntakeBindings(InputStream stream) {
//        this.inputStream = Objects.requireNonNull(stream);
//        this.isPresent = subsystems.intake() != null;
//    }
//
//    public IntakeBindings withRunWithDelays(Trigger run, boolean isIn, Time onTime, Time offTime) {
//        if (!isPresent) {
//            return this;
//        }
//        inputStream.getIsMode().and(run).whileTrue(
//                        subsystems.intake().intake(intakeSpeed, isIn).withTimeout(onTime)
//                                .andThen(Commands.waitSeconds(offTime.in(Seconds))).repeatedly())
//                .onFalse(subsystems.intake().stopIntake());
//        return this;
//    }
//
//    /**
//     * Runs the intake at preset speed, stopping when finished.
//     *
//     * @param runIntake then button to map.
//     * @param isIn      whether to spin in or out.
//     * @return this, for chaining.
//     */
//    public IntakeBindings withRunIntake(Trigger runIntake, boolean isIn) {
//        if (!isPresent) {
//            return this;
//        }
//        inputStream.getIsMode().and(runIntake).whileTrue(subsystems.intake().intake(intakeSpeed, isIn))
//                .onFalse(subsystems.intake().stopIntake());
//        return this;
//    }
//
//    /**
//     * Sets a default command for this subsystem.
//     *
//     * @param defaultCommand the default command to set.
//     * @return this, for chaining.
//     */
//    public IntakeBindings withDefaultCommand(Supplier<Command> defaultCommand) {
//        if (!isPresent) {
//            return this;
//        }
//        inputStream.getIsMode().and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> subsystems.intake().setDefaultCommand(defaultCommand.get())));
//        return this;
//    }
//
//    /**
//     * Leaves IntakeBindings going back to the InputStream.
//     *
//     * @return this InputStream.
//     */
//    public InputStream back() {
//        return this.inputStream;
//    }
//}
