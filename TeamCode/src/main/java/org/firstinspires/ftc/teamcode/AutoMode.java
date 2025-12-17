package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.PixyBlockList;
import org.firstinspires.ftc.teamcode.hardware.PixyCam;

@Autonomous(name = "Auto Mode", group = "Autonomous")
public class AutoMode extends OpMode {
    private PixyCam pixyCam;
    private PixyBlockList greenBalls;
    private PixyBlockList purpleBalls;
    private PixyBlockList aprilTags;
    private DcMotor launcherMotor_01;
    private DcMotor launcherMotor_02;
    private DcMotor intakeMotor;

    @Override
    public void init() {
        // PixyCam initialization
        this.pixyCam = hardwareMap.get(PixyCam.class, "pixy2");
        pixyCam.initialize();
        // Balls
        this.greenBalls = pixyCam.getBiggestBlocks(1);
        this.purpleBalls = pixyCam.getBiggestBlocks(2);
        // AprilTag initialization
        // April Tag Id: 21 GPP
        this.aprilTags = pixyCam.getBiggestBlocks(3);
        // April Tag Id: 22 PGP
        this.aprilTags.addAll(pixyCam.getBiggestBlocks(4));
        // April Tag Id: 23 PPG
        this.aprilTags.addAll(pixyCam.getBiggestBlocks(5));
        // April Tag Id: 20 Blue Alliance Obelisk
        this.aprilTags.addAll(pixyCam.getBiggestBlocks(6));
        // April Tag Id: 24 Blue Alliance Obelisk
        this.aprilTags.addAll(pixyCam.getBiggestBlocks(7));

        // Motor initialization
        this.launcherMotor_01 = hardwareMap.get(DcMotor.class, "launcherMotor_01");
        this.launcherMotor_02 = hardwareMap.get(DcMotor.class, "launcherMotor_02");
        this.intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
    }

    @Override
    public void loop() {
        // Clear old blocks
        greenBalls.clear();
        purpleBalls.clear();
        aprilTags.clear();

        // Add balls
        greenBalls.addAll(pixyCam.getBiggestBlocks(1));
        purpleBalls.addAll(pixyCam.getBiggestBlocks(2));

        // Add april tags
        aprilTags.addAll(pixyCam.getBiggestBlocks(3));
        aprilTags.addAll(pixyCam.getBiggestBlocks(4));
        aprilTags.addAll(pixyCam.getBiggestBlocks(5));
        aprilTags.addAll(pixyCam.getBiggestBlocks(6));
        aprilTags.addAll(pixyCam.getBiggestBlocks(7));

        // Telemetry
        telemetry.addData("Green Balls Detected: ", greenBalls.size());
        telemetry.addData("Purple Balls Detected: ", purpleBalls.size());
        telemetry.update();
    }
}
