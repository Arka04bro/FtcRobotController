package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.security.InvalidParameterException;

public class SubsystemCollection {
    private static SubsystemCollection instance = null;
    private HardwareMap hardwareMap = null;

    // TODO: add intake and release subsystems
    public final DriveTrain driveTrain;


    private SubsystemCollection(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        driveTrain = new DriveTrain(this.hardwareMap);
    }

    /**
     * Возвращает экземпляр SubsystemCollection. Если этот экземпляр еще не существует, он
     * будет создан. В этом случае `hardwareMap` должен быть предоставлен соответствующим объектом,
     * в противном случае он может быть null.
     **/
    public static SubsystemCollection getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            if (hardwareMap == null) {
                throw new InvalidParameterException();
            }

            instance = new SubsystemCollection(hardwareMap);
        }

        return instance;
    }

    public void periodic() {
        driveTrain.periodic();
    }

    public static void deinit() {
        instance = null;
    }
}
