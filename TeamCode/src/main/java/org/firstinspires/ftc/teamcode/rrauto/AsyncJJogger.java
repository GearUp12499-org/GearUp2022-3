package org.firstinspires.ftc.teamcode.rrauto;

import static org.firstinspires.ftc.teamcode.SharedHardware.encoderLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.encoderRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.frontLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.frontRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.turret;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.lib.Supplier;
import org.firstinspires.ftc.teamcode.lib.jobs.Job;
import org.firstinspires.ftc.teamcode.lib.jobs.JobFactory;
import org.firstinspires.ftc.teamcode.lib.jobs.JobManager;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public final class AsyncJJogger {
    private final JobManager mgr;

    public AsyncJJogger(JobManager mgr) {
        this.mgr = mgr;
    }
    Job moveTurretTo(double speed, int target) {
        return moveTurretTo(speed, () -> target);
    }

    // JJogger reimplementation
    Job moveTurretTo(double speed, Supplier<Integer> integerSupplier) {
        // Dummy class to support get/set of data in the Job.
        AtomicBoolean data = new AtomicBoolean(false);

        return mgr.factory
                .manager(mgr)
                .onStart(() -> { /* onStart */
                    data.set(turret.getCurrentPosition() < integerSupplier.get());
                    turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    turret.setTargetPosition(integerSupplier.get());
                    if (turret.getCurrentPosition() < integerSupplier.get())
                        turret.setPower(speed);
                    else if (turret.getCurrentPosition() > integerSupplier.get())
                        turret.setPower(-speed);
                    else {
                        turret.setPower(0);
                    }
                })
                .task(() -> { /* task */
                    if (Math.abs(turret.getCurrentPosition() - integerSupplier.get()) < 150) {
                        turret.setPower(0.2 * (data.get() ? 1 : -1));
                    }
                })
                .completeCondition(() -> { /* completeCondition */
                    return data.get() ? turret.getCurrentPosition() >= integerSupplier.get() : turret.getCurrentPosition() <= integerSupplier.get();
                })
                .onComplete(() -> turret.setPower(0))
                .build();
    }
    Job straight(double speed, double distance) {
        // 1700 ec approx. 1 in

        double adjust = 0.05;
        AtomicInteger targetEncoderCounts = new AtomicInteger((int) (distance * 1700.0));

        return mgr.factory
                .manager(mgr)
                .onStart(
                        () -> targetEncoderCounts.set((int) (distance * 1700.0) + encoderLeft.getCurrentPosition())
                )
                .task(() -> {
                    double realSpeed = speed;
                    if (encoderLeft.getCurrentPosition() / 1700.0 > 41) {
                        realSpeed = 0.2;
                    }
                    if (encoderLeft.getCurrentPosition() > encoderRight.getCurrentPosition()) {
                        frontLeft.setPower(realSpeed - adjust);
                        frontRight.setPower(realSpeed);
                        rearLeft.setPower(realSpeed - adjust);
                        rearRight.setPower(realSpeed);
                    } else if (encoderLeft.getCurrentPosition() < encoderRight.getCurrentPosition()) {
                        frontLeft.setPower(realSpeed + adjust);
                        frontRight.setPower(realSpeed);
                        rearLeft.setPower(realSpeed + adjust);
                        rearRight.setPower(realSpeed);
                    } else if (encoderLeft.getCurrentPosition() == encoderRight.getCurrentPosition()) {
                        frontLeft.setPower(realSpeed);
                        frontRight.setPower(realSpeed);
                        rearLeft.setPower(realSpeed);
                        rearRight.setPower(realSpeed);
                    }
                })
                .completeCondition(() -> encoderLeft.getCurrentPosition() > targetEncoderCounts.get())
                .onComplete(() -> { /* onComplete */
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    rearLeft.setPower(0);
                    rearRight.setPower(0);
                })
                .build();
    }
}
