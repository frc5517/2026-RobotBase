//package frc.robot.inputstream;
//
//import edu.wpi.first.wpilibj.RobotBase;
//import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.robot.InputBuilder;
//import swervelib.simulation.ironmaple.simulation.SimulatedArena;
//
//import java.util.Objects;
//
//import static frc.robot.Telemetry.Publishers.Robot.inputOverride;
//
///**
// * Various Bindings that don't fit into a category yet.
// */
//public class MiscBindings {
//    /**
//     * The {@link InputStream} to attach to.
//     */
//    private final InputStream inputStream;
//
//    public MiscBindings(InputStream stream) {
//        this.inputStream = Objects.requireNonNull(stream);
//    }
//
//    /**
//     * Resets the simulated field.
//     *
//     * @param resetField button to map.
//     * @return this, for chaining.
//     */
//    public MiscBindings resetField(Trigger resetField) {
//        inputStream.getIsMode().and(RobotBase::isSimulation).and(resetField).onTrue(
//                Commands.runOnce(() -> SimulatedArena.getInstance().resetFieldForAuto()));
//        return this;
//    }
//
//    /**
//     * Changes inputs to the given input selection.
//     *
//     * @param bindingType The input to change to.
//     * @param changeInput button to map.
//     * @return this, for chaining.
//     */
//    public MiscBindings withChangeInput(InputBuilder.InputSelections bindingType, Trigger changeInput) {
//        inputStream.getIsMode().and(changeInput).onTrue(Commands.runOnce(() -> inputOverride.set(bindingType.name)));
//        return this;
//    }
//
//    /**
//     * Leaves MiscBindings going back to the InputStream.
//     *
//     * @return this InputStream.
//     */
//    public InputStream back() {
//        return this.inputStream;
//    }
//}