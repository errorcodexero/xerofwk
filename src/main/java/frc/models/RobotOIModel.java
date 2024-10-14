package frc.models;

import org.xero1425.simulator.models.OIBaseModel;
import org.xero1425.simulator.engine.SimulationEngine;

import java.util.Map;

public class RobotOIModel extends OIBaseModel {
    private static final Map<String, Integer> buttonMap = Map.of(
        "eject", 1
    ) ;    

    public RobotOIModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst, buttonMap);
    }
}
