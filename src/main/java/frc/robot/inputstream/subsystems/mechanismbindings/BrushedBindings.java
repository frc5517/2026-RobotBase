//package frc.robot.inputstream.subsystems.mechanismbindings;
//
//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.robot.InputBuilder;
//import frc.robot.inputstream.InputStream;
//import lombok.Getter;
//import lombok.Setter;
//
//import java.util.Objects;
//import java.util.function.Supplier;
//
//import static frc.robot.RobotContainer.subsystems;
//
//public class BrushedBindings {
//    /**
//     * Checks if this subsystem is present, if not, don't bind anything.
//     */
//    @Getter
//    private final boolean isPresent;
//    /**
//     * The {@link InputStream} to attach to.
//     */
//    private final InputStream inputStream;
//
//    /**
//     * The default duty cycle speed to run at.
//     */
//    @Setter @Getter
//    private double dutyCycleSpeed = 1;
//
//    /**
//     * The basic BrushedBindings.
//     * This class holds controls that can be used on ANY brushed mechanism.
//     *
//     * @param stream the {@link InputStream} to attach to.
//     * @param isPresent whether this subsystem is present. [subystems.mech != null]
//     */
//    private BrushedBindings(InputStream stream, boolean isPresent) {
//        this.inputStream = Objects.requireNonNull(stream);
//        this.isPresent = isPresent;
//    }
//
//    /**
//     * Runs the motor at preset speed, stopping when finished.
//     *
//     * @param runDutyCycle then button to map.
//     * @param isForward     whether to spin forward.
//     * @param speed     duty cycle speed to run. [-1, 1]
//     * @return this, for chaining.
//     */
//    public BrushedBindings withRunDutyCycle(Trigger runDutyCycle, boolean isForward, double speed) {
//        if (!isPresent) {
//            return this;
//        }
//        isMode.and(runkicker).whileTrue(subsystems.kicker.runKicker(speed, is))
//                .onFalse(subsystems.kicker.stopKicker());
//        return this;
//    }
//
//    /**
//     * Runs the kicker at preset speed, stopping when finished.
//     *
//     * @param runkicker then button to map.
//     * @param isOut     whether to spin out.
//     * @return this, for chaining.
//     */
//    public InputBuilder.InputStream.KickerBindings withRunKicker(Trigger runkicker, boolean isOut) {
//        if (!isPresent) {
//            return this;
//        }
//        this.withRunKicker(runkicker, isOut, kickerSpeed);
//        return this;
//    }
//
//    /**
//     * Sets a default command for this subsystem.
//     *
//     * @param defaultCommand the default command to set.
//     * @return this, for chaining.
//     */
//    public InputBuilder.InputStream.KickerBindings withDefaultCommand(Supplier<Command> defaultCommand) {
//        if (!isPresent) {
//            return this;
//        }
//        isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> subsystems.kicker.setDefaultCommand(defaultCommand.get())));
//        return this;
//    }
//
//    /**
//     * Leaves KickerBindings going back to the InputStream.
//     *
//     * @return this InputStream.
//     */
//    public InputBuilder.InputStream back() {
//        return InputBuilder.InputStream.this;
//    }
//}
//}
