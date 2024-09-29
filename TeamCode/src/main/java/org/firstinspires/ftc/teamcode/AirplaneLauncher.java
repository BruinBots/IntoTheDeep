
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;


import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Airplane Launcher", group="Iterative OpMode")
public class AirplaneLauncher extends OpMode
{
    // robot
    DroneMap bot;
    double pos = 0.4;

    double TURRET_SPEED = 0.3;


    @Override
    public void init() {
        bot = new DroneMap(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        if ((gamepad1.dpad_up || gamepad1.left_stick_y > 0.1 || gamepad1.right_stick_y > 0.1) && bot.droneRotateServo.getPosition() < Drone.MIN_ROTATE_POS) {
            pos += 0.005;
        }
        else if ((gamepad1.dpad_down || gamepad1.left_stick_y < -0.1 || gamepad1.right_stick_y < -0.1) && bot.droneRotateServo.getPosition() > Drone.MAX_ROTATE_POS)
        {
            pos -= 0.005;
        }
        bot.droneRotateServo.setPosition(pos);

        if (gamepad1.dpad_left || gamepad1.left_stick_x < -0.1 || gamepad1.right_stick_x < -0.1) {
            bot.drone.setTurret(0.5 - TURRET_SPEED);
        }
        else if (gamepad1.dpad_right || gamepad1.left_stick_x > 0.1 || gamepad1.right_stick_x > 0.1) {
            bot.drone.setTurret(0.5 + TURRET_SPEED);
        }
        else {
            bot.drone.setTurret(0.5);
        }

        // drone launcher
        if (gamepad1.y || (gamepad1.left_bumper && gamepad1.right_bumper)) {
            bot.drone.launchWithRotation(pos); // 0.4
        }
        bot.drone.loop();

        try {
            sleep(20);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}