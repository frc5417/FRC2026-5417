package frc.robot.helpers;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public final class ControllerHelper {
    // /**
    // * General use method to get the value of an axis value (because duplicity
    // sucks)
    // *
    // * @param supplier
    // * @return
    // */
    // private static final double axisValue(Supplier<Double> supplier) {
    // return supplier.get();
    // }

    public static final class Manipulator {
        public static final XboxController kManipulator = new XboxController(1);

        public static final boolean leftBumper() {
            return kManipulator.getLeftBumperButton();
        }

        public static final double leftTrigger() {
            if (Math.abs(kManipulator.getLeftTriggerAxis()) < Constants.OperatorConstants.kDeadband) {
                return 0;
            }
            return kManipulator.getLeftTriggerAxis();
        }

        public static final double rightTrigger() {
            if (Math.abs(kManipulator.getRightTriggerAxis()) < Constants.OperatorConstants.kDeadband) {
                return 0;
            }
            return kManipulator.getRightTriggerAxis();
        }
    }
}
