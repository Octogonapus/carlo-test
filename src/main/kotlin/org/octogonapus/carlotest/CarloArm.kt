/*
 * This file is part of carlo-test.
 *
 * carlo-test is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * carlo-test is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with carlo-test.  If not, see <https://www.gnu.org/licenses/>.
 */
package org.octogonapus.carlotest

import arrow.core.Either
import arrow.core.left
import arrow.core.right
import com.google.common.collect.ImmutableList
import com.neuronrobotics.bowlerkernel.hardware.Script
import com.neuronrobotics.bowlerkernel.hardware.device.BowlerDeviceFactory
import com.neuronrobotics.bowlerkernel.hardware.device.deviceid.DefaultConnectionMethods
import com.neuronrobotics.bowlerkernel.hardware.device.deviceid.DefaultDeviceTypes
import com.neuronrobotics.bowlerkernel.hardware.device.deviceid.DeviceId
import com.neuronrobotics.bowlerkernel.hardware.deviceresource.resourceid.DefaultAttachmentPoints
import com.neuronrobotics.bowlerkernel.hardware.deviceresource.unprovisioned.UnprovisionedServoFactory
import com.neuronrobotics.bowlerkernel.kinematics.base.DefaultKinematicBase
import com.neuronrobotics.bowlerkernel.kinematics.base.baseid.SimpleKinematicBaseId
import com.neuronrobotics.bowlerkernel.kinematics.closedloop.JointAngleController
import com.neuronrobotics.bowlerkernel.kinematics.closedloop.NoopBodyController
import com.neuronrobotics.bowlerkernel.kinematics.limb.DefaultLimb
import com.neuronrobotics.bowlerkernel.kinematics.limb.limbid.SimpleLimbId
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.DefaultLink
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.DhParam
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.Link
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.LinkType
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.toFrameTransformation
import com.neuronrobotics.bowlerkernel.kinematics.motion.BasicMotionConstraints
import com.neuronrobotics.bowlerkernel.kinematics.motion.ForwardKinematicsSolver
import com.neuronrobotics.bowlerkernel.kinematics.motion.FrameTransformation
import com.neuronrobotics.bowlerkernel.kinematics.motion.InverseKinematicsSolver
import com.neuronrobotics.bowlerkernel.kinematics.motion.MotionConstraints
import com.neuronrobotics.bowlerkernel.kinematics.motion.NoopInertialStateEstimator
import com.neuronrobotics.bowlerkernel.kinematics.motion.plan.DefaultLimbMotionPlanFollower
import com.neuronrobotics.bowlerkernel.util.Limits
import org.octogonapus.ktguava.collections.immutableListOf
import org.octogonapus.ktguava.collections.immutableMapOf
import org.octogonapus.ktguava.collections.toImmutableList
import java.net.InetAddress
import javax.inject.Inject

class CarloArm
@Inject constructor(
    deviceFactory: BowlerDeviceFactory,
    private val servoFactory: UnprovisionedServoFactory
) : Script() {

    private val relaxedMotionConstraints = BasicMotionConstraints(1, 10, 100, 100)

    private val links: ImmutableList<Link> = immutableListOf(
        DefaultLink(
            LinkType.Rotary,
            DhParam(44.45, 0, 196.85, 0),
            Limits(180, -180),
            NoopInertialStateEstimator
        ),
        DefaultLink(
            LinkType.Rotary,
            DhParam(38.1, 0, 273.05, 0),
            Limits(180, -180),
            NoopInertialStateEstimator
        )
    )

    private val dhParams = links.map { it.dhParam }.toImmutableList()

    private val limb1Id = SimpleLimbId("carlo-arm-limb1")

    private val fkEngine = object : ForwardKinematicsSolver {
        override fun solveChain(currentJointAngles: ImmutableList<Double>) =
            dhParams.mapIndexed { index, dhParam ->
                dhParam.copy(theta = currentJointAngles[index] + dhParam.theta)
            }.toFrameTransformation()
    }

    private val ikEngine = object : InverseKinematicsSolver {
        override fun solveChain(
            currentJointAngles: ImmutableList<Double>,
            targetFrameTransform: FrameTransformation
        ): ImmutableList<Double> {
            // TODO: Real IK
            return currentJointAngles.mapIndexed { index, _ -> index.toDouble() }.toImmutableList()
        }
    }

    private val device = deviceFactory.makeBowlerDevice(
        DeviceId(
            DefaultDeviceTypes.Esp32wroom32,
            DefaultConnectionMethods.InternetAddress(
                InetAddress.getByAddress(
                    listOf(192, 168, 4, 1).map { it.toByte() }.toByteArray()
                )
            )
        )
    ).fold({ throw IllegalStateException(it) }, { it })

    override fun runScript(args: ImmutableList<Any?>): Either<String, Any?> {
        device.connect().mapLeft { return it.left() }

        val servos = links.mapIndexed { index, _ ->
            servoFactory.makeUnprovisionedServo(
                device,
                DefaultAttachmentPoints.Pin((32 + index).toByte())
            ).fold({ return it.left() }, { it })
        }.map {
            device.add(it).fold({ return it.left() }, { it })
        }.toImmutableList()

        val limb1 = DefaultLimb(
            limb1Id,
            links,
            fkEngine,
            ikEngine,
            CarloLimbMotionPlanGenerator(ikEngine),
            DefaultLimbMotionPlanFollower(),
            links.mapIndexed { index, _ ->
                object : JointAngleController {
                    override fun getCurrentAngle() = servos[index].read().also {
                        println("read $it")
                    }

                    override fun setTargetAngle(angle: Double, motionConstraints: MotionConstraints) {
                        println("write $angle")
                        servos[index].write(angle)
                    }
                }
            }.toImmutableList(),
            NoopInertialStateEstimator
        )

        val base = DefaultKinematicBase(
            SimpleKinematicBaseId("carlo-arm-base"),
            immutableListOf(limb1),
            immutableMapOf(limb1Id to FrameTransformation.identity),
            NoopBodyController
        )

        println("Setting transform")
        limb1.setDesiredTaskSpaceTransform(
            dhParams.toFrameTransformation(),
            relaxedMotionConstraints
        )

        while (limb1.isMovingToTaskSpaceTransform) {
            Thread.sleep(10)
        }

        return Unit.right()
    }

    override fun stopScript() {
        device.disconnect()
    }
}
