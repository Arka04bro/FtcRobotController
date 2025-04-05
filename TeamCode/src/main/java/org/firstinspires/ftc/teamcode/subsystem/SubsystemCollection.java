package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.security.InvalidParameterException;

public class SubsystemCollection {
    private static SubsystemCollection instance = null;
    private HardwareMap hardwareMap = null;

    // NOTE: Maybe we need to separate the subsystems
    public final DriveTrain driveTrain;
    public final Slider slider;
//    public final SliderRotation sliderRotation;
    public final Claw claw;
    public final Vision vision;

    private SubsystemCollection(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        vision = new Vision(this.hardwareMap);
        driveTrain = new DriveTrain(this.hardwareMap);
        slider = new Slider(this.hardwareMap);
//        sliderRotation = new SliderRotation(this.hardwareMap);
        claw = new Claw(this.hardwareMap);
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

    public static void deInit() {
        instance = null;
    }
}
