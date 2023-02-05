package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import org.firstinspires.ftc.teamcode.drive.opmode.*;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

import java.util.Arrays;
import java.util.List;

public final class TuningOpModes {
    public static final Class<?> DRIVE_CLASS = MecanumDrive.class;

    public static final String GROUP = "quickstart";
    public static final boolean DISABLED = false;

    private TuningOpModes() {

    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;

        List<Class<? extends OpMode>> opModes = Arrays.asList(
                AutomaticFeedforwardTuner.class,
                BackAndForth.class,
                DriveVelocityPIDTuner.class,
                FollowerPIDTuner.class,
                LocalizationTest.class,
                ManualFeedforwardTuner.class,
                MaxAngularVeloTuner.class,
                MaxVelocityTuner.class,
                MotorDirectionDebugger.class,
                SplineTest.class,
                StrafeTest.class,
                StraightTest.class,
                TrackingWheelForwardOffsetTuner.class,
                TrackingWheelLateralDistanceTuner.class,
                TrackWidthTuner.class,
                TurnTest.class
                );

        for (Class<? extends OpMode> o : opModes) {
            manager.register(new OpModeMeta.Builder()
                    .setName(o.getSimpleName())
                    .setGroup(GROUP)
                    .setFlavor(OpModeMeta.Flavor.TELEOP)
                    .build(), o);
        }
    }
}
