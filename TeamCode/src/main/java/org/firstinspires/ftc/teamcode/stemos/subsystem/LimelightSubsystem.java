/*
Copyright (c) 2024 Limelight Vision

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.stemos.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.stemos.Constants;

/*
 * This OpMode illustrates how to use the Limelight3A Vision Sensor.
 *
 * @see <a href="https://limelightvision.io/">Limelight</a>
 *
 * Notes on configuration:
 *
 *   The device presents itself, when plugged into a USB port on a Control Hub as an ethernet
 *   interface.  A DHCP server running on the Limelight automatically assigns the Control Hub an
 *   ip address for the new ethernet interface.
 *
 *   Since the Limelight is plugged into a USB port, it will be listed on the top level configuration
 *   activity along with the Control Hub Portal and other USB devices such as webcams.  Typically
 *   serial numbers are displayed below the device's names.  In the case of the Limelight device, the
 *   Control Hub's assigned ip address for that ethernet interface is used as the "serial number".
 *
 *   Tapping the Limelight's name, transitions to a new screen where the user can rename the Limelight
 *   and specify the Limelight's ip address.  Users should take care not to confuse the ip address of
 *   the Limelight itself, which can be configured through the Limelight settings page via a web browser,
 *   and the ip address the Limelight device assigned the Control Hub and which is displayed in small text
 *   below the name of the Limelight on the top level configuration screen.
 */

public class LimelightSubsystem {

    private final Limelight3A limelight;
    private final OpMode opMode;
    private LLResult result = null;
    public LimelightSubsystem(OpMode opMode)
    {
        this.opMode = opMode;
        limelight = opMode.hardwareMap.get(Limelight3A.class, Constants.LimelightNames.LIMELIGHT_NAME);

        opMode.telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();
    }

    public void getStatusFromLimelight()
    {
        LLStatus status = limelight.getStatus();
        opMode.telemetry.addData("Name", "%s",
                status.getName());
        opMode.telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        opMode.telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());
    }

    public void updateCamera()
    {
        result = limelight.getLatestResult();
    }

    public void updateCameraWithHeading(double yaw)
    {
        limelight.updateRobotOrientation(yaw);
        result = limelight.getLatestResult();
    }

    public void getLatency()
    {
        double captureLatency = result.getCaptureLatency();
        double targetingLatency = result.getTargetingLatency();
        double parseLatency = result.getParseLatency();
        opMode.telemetry.addData("LL Latency", captureLatency + targetingLatency);
        opMode.telemetry.addData("Parse Latency", parseLatency);
        opMode.telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
    }

    public boolean isValid()
    {
        return result.isValid();
    }

    public boolean notNull()
    {
        return result != null;
    }

    public double readTx()
    {
        return result.getTx();
    }

    public double readTy()
    {
        return result.getTy();
    }

    public Pose3D getRobotPose()
    {
        return result.getBotpose();
    }

    public Pose3D getRobotPoseMT2()
    {
        return result.getBotpose_MT2();
    }
}
