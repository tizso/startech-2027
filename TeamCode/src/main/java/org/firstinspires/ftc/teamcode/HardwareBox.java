package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static com.pedropathing.ivy.commands.Commands.instant;

public class HardwareBox {

    // Hardver eszközök
    public DcMotor intake = null;
    public DcMotorEx outtake = null;
    public DcMotorEx turet = null;
    public Servo servoR = null;
    public Servo servoL = null;
    public Servo gate = null;

    HardwareMap hwMap = null;

    // TeleOp szervó konstansok
    public static final double SERVO_L_POLLEN = 0.30;
    public static final double SERVO_R_POLLEN = 0.70;
    public static final double SERVO_L_NECTAR = 0.15;
    public static final double SERVO_R_NECTAR = 0.85;
    public static final double GATE_CLOSED = 0.0;
    public static final double GATE_OPEN = 0.45;

    // Automata lőtorony változók
    private boolean turetTrackingActive = false;
    public double targetX = 72.0;
    public double targetY = 72.0;
    private final double TURET_TICKS_PER_RADIAN = 1497.0;

    public HardwareBox() {}

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // BIZTONSÁGI BLOKK: Ha a telefonon nincs bekonfigurálva az adott név,
        // a try-catch elkapja a hibát, null-on hagyja a változót, de NEM ENGEDI LEFAGYNI a robotot!
        try { intake = hwMap.get(DcMotor.class, "intake"); intake.setDirection(DcMotorSimple.Direction.REVERSE); intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); } catch (Exception e) { intake = null; }
        try { outtake = hwMap.get(DcMotorEx.class, "outtake"); outtake.setDirection(DcMotorEx.Direction.FORWARD); outtake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); } catch (Exception e) { outtake = null; }
        try {
            turet = hwMap.get(DcMotorEx.class, "turet");
            turet.setDirection(DcMotorEx.Direction.FORWARD);
            turet.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            turet.setTargetPosition(0);
            turet.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            turet.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) { turet = null; }

        try { servoR = hwMap.get(Servo.class, "servoR"); } catch (Exception e) { servoR = null; }
        try { servoL = hwMap.get(Servo.class, "servoL"); } catch (Exception e) { servoL = null; }
        try { gate = hwMap.get(Servo.class, "gate"); } catch (Exception e) { gate = null; }

        // Csak akkor állítjuk be a kezdőpozíciót, ha a szervók fizikailag léteznek
        prepareForPollen();
    }

    // =========================================================================
    // CRASH-SAFE VÁLTOZTATÁSOK: Minden parancs előtt ellenőrizzük, hogy létezik-e a motor!
    // =========================================================================
    public void prepareForPollen() {
        if (servoL != null && servoR != null) {
            servoL.setPosition(SERVO_L_POLLEN);
            servoR.setPosition(SERVO_R_POLLEN);
        }
    }

    public void prepareForNectar() {
        if (servoL != null && servoR != null) {
            servoL.setPosition(SERVO_L_NECTAR);
            servoR.setPosition(SERVO_R_NECTAR);
        }
    }

    public Command startIntake() {
        return instant(() -> { if (intake != null) intake.setPower(1.0); });
    }
    public Command stopIntake() {
        return instant(() -> { if (intake != null) intake.setPower(0.0); });
    }
    public Command startOuttake(double power) {
        return instant(() -> { if (outtake != null) outtake.setPower(power); });
    }
    public Command stopOuttake() {
        return instant(() -> { if (outtake != null) outtake.setPower(0.0); });
    }
    public Command openGate() {
        return instant(() -> { if (gate != null) gate.setPosition(GATE_OPEN); });
    }
    public Command closeGate() {
        return instant(() -> { if (gate != null) gate.setPosition(GATE_CLOSED); });
    }
    public Command enableTuretTracking() { return instant(() -> turetTrackingActive = true); }
    public Command disableTuretTracking() {
        return Command.build().setStart(() -> {
            turetTrackingActive = false;
            if (turet != null) turet.setTargetPosition(0);
        });
    }

    public Command setTargetToAudienceCell(boolean isBlue) {
        return Command.build().setStart(() -> {
            targetX = 72.0;
            // A BioBuzz gyári adatok alapján számolt eltolás a pálya középvonalától (inch)
            targetY = isBlue ? (72.0 + 12.75) : (72.0 - 12.75);
        });
    }

    /** Parancs, ami átállítja a lőtornyot a hátsó (Backstage) cellára */
    public Command setTargetToBackstageCell(boolean isBlue) {
        return Command.build().setStart(() -> {
            targetX = 72.0;
            targetY = isBlue ? (72.0 - 12.75) : (72.0 + 12.75);
        });
    }

    public void updateTuret(double robotX, double robotY, double robotHeadingRad) {
        // Ha a turet motor még nem létezik fizikailag, a matematika lefut, de nem hív meg hardvert, így nincs crash!
        if (!turetTrackingActive || turet == null) return;

        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        double absoluteTargetAngle = Math.atan2(deltaY, deltaX);
        double relativeTargetAngle = absoluteTargetAngle - robotHeadingRad;

        relativeTargetAngle = Math.atan2(Math.sin(relativeTargetAngle), Math.cos(relativeTargetAngle));

        double maxLimitRad = Math.toRadians(90.0);
        double minLimitRad = Math.toRadians(-90.0);
        if (relativeTargetAngle > maxLimitRad) relativeTargetAngle = maxLimitRad;
        else if (relativeTargetAngle < minLimitRad) relativeTargetAngle = minLimitRad;

        int targetTick = (int) (relativeTargetAngle * TURET_TICKS_PER_RADIAN);

        turet.setTargetPosition(targetTick);
        turet.setPower(0.65);
    }
}
