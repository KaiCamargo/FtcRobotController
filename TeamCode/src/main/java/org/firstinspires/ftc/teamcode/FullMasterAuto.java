package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Config
@Autonomous(name = "DECODE: Full Omni-Ball Run", group = "Autonomous")
public class FullMasterAuto extends LinearOpMode {

    // --- DASHBOARD CONFIGURATION (Adjustable via Laptop) ---
    public static double BALL_1_X = 24, BALL_1_Y = 24;
    public static double BALL_2_X = 48, BALL_2_Y = 0;
    public static double BALL_3_X = 60, BALL_3_Y = -12;
    public static double LAUNCH_X = 15, LAUNCH_Y = 0;
    public static double ALLIANCE_Y_FACTOR = 1.0; // 1 for Red, -1 for Blue

    private Follower follower;
    private FtcDashboard dashboard;
    private String currentAlliance = "DETECTING...";

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // Fast updates
        limelight.start();
        // 1. Setup Pedro Pathing (Linked to Pinpoint in Constants)
        follower = Constants.createFollower(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // --- INITIALIZATION LOOP ---
        while (!isStarted() && !isStopRequested()) {
            localizeWithLimelight(); // Get "Off-Rip" position
            updateAlliance();        // Check for Red vs Blue tags
            drawFieldOnDashboard();  // Show robot on your laptop screen

            telemetry.addData("Alliance", currentAlliance);
            telemetry.addData("Current Pose", follower.getPose().toString());
            telemetry.update();
        }

        waitForStart();
        localizeWithLimelight();
        // --- THE BALL RUN SEQUENCE ---

        // Ball 1
        driveTo(new Pose(BALL_1_X, BALL_1_Y * ALLIANCE_Y_FACTOR, 0));
        intakeArtifact();

        // Ball 2
        driveTo(new Pose(BALL_2_X, BALL_2_Y * ALLIANCE_Y_FACTOR, 0));
        intakeArtifact();

        // Ball 3
        driveTo(new Pose(BALL_3_X, BALL_3_Y * ALLIANCE_Y_FACTOR, Math.toRadians(45 * ALLIANCE_Y_FACTOR)));
        intakeArtifact();

        // Go to Launch Zone
        driveTo(new Pose(LAUNCH_X, LAUNCH_Y * ALLIANCE_Y_FACTOR, Math.toRadians(180)));

        // Final sanity check with Limelight before firing
        localizeWithLimelight();
        executeLaunch();
    }

    /**
     * Moves the robot to a target.
     * Uses Pinpoint for precision and Limelight for drift correction.
     */
    private void driveTo(Pose target) {
        follower.followPath(follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), target.getPose()))
                .setConstantHeadingInterpolation(target.getHeading())
                .build(), true);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            // Only relocalize if moving slowly to avoid camera blur
            if (follower.getVelocity().getMagnitude() < 0.1) {

                localizeWithLimelight();
            }

            drawFieldOnDashboard();
        }
    }

    /**
     * Logic to detect alliance based on AprilTag IDs
     */
    private void updateAlliance() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Limelight can see multiple tags, so it returns a list
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            for (LLResultTypes.FiducialResult f : fiducials) {
                int id = f.getFiducialId(); // THIS is the method you need

                if (id >= 11 && id <= 13) {
                    currentAlliance = "RED";
                    ALLIANCE_Y_FACTOR = 1.0;
                } else if (id >= 14 && id <= 16) {
                    currentAlliance = "BLUE";
                    ALLIANCE_Y_FACTOR = -1.0;
                }
            }
        }
    }

    /**
     * Official FTC Localization logic:
     * Pulls Botpose (Meters) -> Converts to Pedro (Inches)
     */
    private void localizeWithLimelight() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            // botpose is already a Pose3D object in the FTC driver!
            // result.getBotpose() returns field-centric coordinates
            Pose3D pose = result.getBotpose();

            if (pose != null) {
                // Convert to Pedro Pathing Pose (Meters to Inches)
                Pose globalPose = new Pose(
                        pose.getPosition().x * 39.37,
                        pose.getPosition().y * 39.37,
                        Math.toRadians(result.getBotpose().getOrientation().getYaw())
                );
                follower.setPose(globalPose);
            }
        }
    }

    /**
     * Draws the field map and robot on your laptop window
     */
    private void drawFieldOnDashboard() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5"); // Blue Robot

        Pose p = follower.getPose();
        packet.fieldOverlay().strokeRect(p.getX(), p.getY(), 18, 18); // Robot square
        packet.fieldOverlay().strokeLine(p.getX(), p.getY(),
                p.getX() + Math.cos(p.getHeading()) * 12,
                p.getY() + Math.sin(p.getHeading()) * 12); // Direction indicator

        dashboard.sendTelemetryPacket(packet);
    }

    private void intakeArtifact() {
        // TDB: Add your intake motor code
        sleep(500);
    }

    private void executeLaunch() {
        // TDB: Add your launch mechanism code
        telemetry.addLine("SHOT FIRED!");
        telemetry.update();
        sleep(1000);
    }
}