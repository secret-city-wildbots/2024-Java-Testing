package frc.robot.Utility;

import frc.robot.Dashboard;

public class ActuatorInterlocks {
    private static String testingActuator;
    private static double testingPeriod;
    private static double testingValue;

    public static double TAI_Motors(String actuatorName, double normalOutput){
        testingActuator = Dashboard.testActuatorName.get();
        testingPeriod = Dashboard.testActuatorPeriod.get();
        testingValue = Dashboard.testActuatorValue.get();
        if (testingActuator.equals("No_Test")) {
            return normalOutput;
        }
        else if (testingActuator.equals(actuatorName)) {
            if (testingPeriod == 0){
                return testingValue;
            } else {
                return testingValue * Math.sin((double)System.currentTimeMillis() /1000.0 * Math.PI / testingPeriod);
            }
        }
        else {
            return 0;
        }
    }

}
