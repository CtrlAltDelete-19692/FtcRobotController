package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "KingBobAutonomous")
public class KingBobAutonomus extends LinearOpMode {

    private Hardware hw;
    private int teamTagId = 20;

    private Drive drive;
    private Launcher launcher;
    private Intake intake;

    private static final String[] AUTO_PROGRAMS = { "Back", "Front" };
    private int autoProgramIndex = 1;

    private static final long FOOT_MS = 415;

    private boolean dpadPressedLast = false;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean isBlue = true;
        int pauseTimeSeconds = 9;

        while (!isStarted()) {
            // Team color selection
            if (gamepad1.x) isBlue = true;
            if (gamepad1.b) isBlue = false;

            // Auto program selection (D-pad left/right)
//            boolean dpadLRPressed = gamepad1.dpad_left || gamepad1.dpad_right;
//            if (dpadLRPressed && !dpadLRPressedLast) {
//                if (gamepad1.dpad_right) {
//                    autoProgramIndex++;
//                    if (autoProgramIndex >= AUTO_PROGRAMS.length) {
//                        autoProgramIndex = 0;
//                    }
//                } else if (gamepad1.dpad_left) {
//                    autoProgramIndex--;
//                    if (autoProgramIndex < 0) {
//                        autoProgramIndex = AUTO_PROGRAMS.length - 1;
//                    }
//                }
//            }
//            dpadLRPressedLast = dpadLRPressed;

            if (gamepad1.dpad_left) {
                autoProgramIndex = 0; // Back
            } else if (gamepad1.dpad_right) {
                autoProgramIndex = 1; // Front
            }

            boolean dpadPressed = gamepad1.dpad_up || gamepad1.dpad_down;
            if (dpadPressed && !dpadPressedLast) {
                if (gamepad1.dpad_up) {
                    pauseTimeSeconds += 1;
                }
                if (gamepad1.dpad_down) {
                    pauseTimeSeconds -= 1;
                }
            }
            dpadPressedLast = dpadPressed;
            if (pauseTimeSeconds >= 10) {
                pauseTimeSeconds = 10;
            } else if (pauseTimeSeconds < 0) {
                pauseTimeSeconds = 0;
            }

            telemetry.addLine("Team Color");
            telemetry.addLine((isBlue ? "\uD83D\uDFE6 Blue" : "\uD83D\uDFE5 Red") + "     (X or B)");
            telemetry.addLine();

            telemetry.addLine("Auto Program");
            telemetry.addLine(AUTO_PROGRAMS[autoProgramIndex] + "     (D-Pad Left / Right)");
            telemetry.addLine();

            telemetry.addLine("Pause Time     (D-Pad Up / D-Pad Down)");
            telemetry.addLine(String.format("%d seconds", pauseTimeSeconds));
            telemetry.update();
        }

        hw = new Hardware();
        hw.setup(hardwareMap);

        int pipeline = 0; // Blue
        int direction = -1; // We program our moves based on Red, and multiple by direction (-1) to reverse rotation / strafing for blue
        teamTagId = 20;
        if (! isBlue) {
            pipeline = 1; // Red
            direction = 1;
            teamTagId = 24;
        }
        drive = new Drive(hw, hardwareMap, "King Bob");

        waitForStart();

        telemetry.update(); // Clear previous telemetry data (selection screen)

        String selectedProgram = AUTO_PROGRAMS[autoProgramIndex];

        if (opModeIsActive()) {

            // Pause out of the way til the end of auto
            driveForward(FOOT_MS * 3, direction);
            pause(pauseTimeSeconds * 1000);

            // Get out of shooting triangle
            driveBackward(FOOT_MS * 3.5, direction);
            strafeRight(1100, direction);
            rotateRight(400, direction);

            stopAllMotors();
        }
    }

    private void driveForTime(double strafe, double forward, double rotate, double durationMs, int direction) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < durationMs) {
            drive.driveCommand(strafe * direction, forward, rotate * direction);
            idle();
        }

        driveStop();
    }

    private void driveForward(double durationMs, int direction) {
        driveForTime(0, 1, 0, durationMs, direction);
    }

    private void driveBackward(double durationMs, int direction) {
        driveForTime(0, -1, 0, durationMs, direction);
    }

    private void rotateLeft(double durationMs, int direction) {
        driveForTime(0, 0, -1, durationMs, direction);
    }

    private void rotateRight(double durationMs, int direction) {
        driveForTime(0, 0, 1, durationMs, direction);
    }

    private void strafeLeft(double durationMs, int direction) {
        driveForTime(-1, 0, 0, durationMs, direction);
    }

    private void strafeRight(double durationMs, int direction) {
        driveForTime(1, 0, 0, durationMs, direction);
    }

    private void sleep(double durationMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < durationMs) {
            idle();
        }

        driveStop();
    }

    private void pause(long durationMs) {
        long start = System.currentTimeMillis();

        while (opModeIsActive()) {
            long elapsedMs   = System.currentTimeMillis() - start;
            long remainingMs = durationMs - elapsedMs;

            if (remainingMs <= 0) {
                break;
            }

            double remainingSec = remainingMs / 1000.0;

            telemetry.addData("Pause", "%.1f seconds left", remainingSec);
            telemetry.update();

            idle();
        }
    }

    private void driveStop() {
        drive.driveCommand(0, 0, 0);
    }


    private void centerOnTag(long durationMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < durationMs) {
            double rotate = drive.getAutoRotate(0, teamTagId);  // re-sample tx each loop
            drive.driveCommand(0, 0, rotate);
            idle();
        }

        driveStop();
    }

    private void stopAllMotors() {
        driveStop();
    }
}
