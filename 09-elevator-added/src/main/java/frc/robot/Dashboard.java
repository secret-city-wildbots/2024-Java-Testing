package frc.robot;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringSubscriber;

public class Dashboard {
    @SuppressWarnings("unused")
    private NetworkTableInstance inst;
    @SuppressWarnings("unused")
    private NetworkTable table;

    public static StringArrayPublisher legalActuatorNames;

    public static StringSubscriber testActuatorName;
    public static DoubleSubscriber testActuatorValue;
    public static DoubleSubscriber testActuatorPeriod;


    public Dashboard(){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("SmartDashboard");

        legalActuatorNames = table.getStringArrayTopic("Legal_Actuator_Names").publish();

        testActuatorName = table.getStringTopic("Test_Actuator_Name").subscribe("");
        testActuatorValue = table.getDoubleTopic("Test_Actuator_Value").subscribe(0.0);
        testActuatorPeriod = table.getDoubleTopic("Test_Actuator_Period").subscribe(0.0);
    }
}
