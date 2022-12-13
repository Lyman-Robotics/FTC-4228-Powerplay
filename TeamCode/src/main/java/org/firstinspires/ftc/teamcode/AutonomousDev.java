package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Autonomous Dev", group = "Autonomous Dev")
public class AutonomousDev extends LinearOpMode {

  @Override
  public void runOpMode() {
    // Initialize the hardware variables.
    RobotClass robot = new RobotClass(hardwareMap, false);

    // ! Runs upon initialization
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    robot.BRDrive.setDirection(DcMotor.Direction.FORWARD);
    robot.FLDrive.setDirection(DcMotor.Direction.REVERSE);
    robot.FRDrive.setDirection(DcMotor.Direction.REVERSE);
    robot.BLDrive.setDirection(DcMotor.Direction.FORWARD);

    boolean canAWork = true;
    boolean canBWork = false;
    int segmentAmount = 0;
    boolean running = false;
    ElapsedTime segmentTime = new ElapsedTime();
    String segmentEnd = "";

    // ! Runs until the end of the match after play is pressed
    waitForStart();
    robot.timeElapsed.reset();

    while (opModeIsActive()) {
      if (gamepad1.a && canAWork) {
        canAWork = false;
        canBWork = true;
        running = true;

        segmentAmount++;
        segmentTime.reset();
      }

      if (gamepad1.b && canBWork) {
        canAWork = true;
        canBWork = false;
        running = false;

        segmentEnd = segmentTime.toString();
        robot.stopDrive();
      }

      if (running) {
        if (gamepad1.left_stick_y < 0) {
          robot.setDrivePower(0.25, 0.25, 0.25, 0.25);
        } else if (gamepad1.left_stick_y > 0) {
          robot.setDrivePower(-0.25, -0.25, -0.25, -0.25);
        } else if (gamepad1.left_stick_x > 0) {
          robot.omnidrive(0.5, (Math.PI / 2), 0);
        } else if (gamepad1.left_stick_x < 0) {
          robot.omnidrive(-0.5, (Math.PI / 2), 0);
        } else {
          robot.setDrivePower(0, 0, 0, 0);
        }
      }

      telemetry.addData(
        "Segment " + segmentAmount + " time",
        segmentTime.toString()
      );
      telemetry.addData("Running", running);
      telemetry.addData("Segment End", segmentEnd);
      telemetry.update();
    }
  }
}
