package robot.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class BulkReadSubsystem {
    private List<LynxModule> allHubs;
    public BulkReadSubsystem(HardwareMap hardwareMap) {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void update() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}
