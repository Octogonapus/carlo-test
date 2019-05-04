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

import arrow.core.extensions.either.monad.binding
import arrow.core.flatMap
import arrow.core.getOrHandle
import com.google.inject.Injector
import com.neuronrobotics.bowlerkernel.hardware.Script
import com.neuronrobotics.bowlerkernel.hardware.device.BowlerDevice
import com.neuronrobotics.bowlerkernel.hardware.device.BowlerDeviceFactory
import com.neuronrobotics.bowlerkernel.hardware.device.deviceid.DefaultConnectionMethods
import com.neuronrobotics.bowlerkernel.hardware.device.deviceid.DefaultDeviceTypes
import com.neuronrobotics.bowlerkernel.hardware.device.deviceid.DeviceId
import com.neuronrobotics.bowlerkernel.hardware.deviceresource.provisioned.DigitalState
import com.neuronrobotics.bowlerkernel.hardware.deviceresource.resourceid.DefaultAttachmentPoints
import com.neuronrobotics.bowlerkernel.hardware.deviceresource.unprovisioned.UnprovisionedDigitalOutFactory
import com.neuronrobotics.bowlerkernel.hardware.deviceresource.unprovisioned.UnprovisionedServoFactory
import com.neuronrobotics.bowlerkernel.kinematics.base.DefaultKinematicBase
import com.neuronrobotics.bowlerkernel.kinematics.base.baseid.SimpleKinematicBaseId
import com.neuronrobotics.bowlerkernel.kinematics.closedloop.NoopBodyController
import com.neuronrobotics.bowlerkernel.kinematics.limb.DefaultLimb
import com.neuronrobotics.bowlerkernel.kinematics.limb.limbid.SimpleLimbId
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.toFrameTransformation
import com.neuronrobotics.bowlerkernel.kinematics.motion.BasicMotionConstraints
import com.neuronrobotics.bowlerkernel.kinematics.motion.FrameTransformation
import com.neuronrobotics.bowlerkernel.kinematics.motion.NoopInertialStateEstimator
import com.neuronrobotics.bowlerkernel.kinematics.motion.plan.DefaultLimbMotionPlanFollower
import org.jlleitschuh.guice.getInstance
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.fail
import org.octogonapus.ktguava.collections.immutableListOf
import org.octogonapus.ktguava.collections.immutableMapOf
import org.octogonapus.ktguava.collections.toImmutableList
import java.net.InetAddress

internal class CarloArmTest {

    private val limb1Id = SimpleLimbId("carlo-arm-limb1")

    private val ikEngine = CarloIK()

    @Test
    fun `test homing with real arm`() {
        val injector = Script.makeScriptInjector().createChildInjector(Script.getDefaultModules())

        val device = getDevice(injector)

        val servoFactory = injector.getInstance<UnprovisionedServoFactory>()
        val servos = seaArmLinks.mapIndexed { index, _ ->
            servoFactory.makeUnprovisionedServo(
                device,
                DefaultAttachmentPoints.Pin((32 + index).toByte())
            ).flatMap { device.add(it) }.getOrHandle { fail(it) }
        }.toImmutableList()

        val limb1 = DefaultLimb(
            limb1Id,
            seaArmLinks,
            CarloFK(),
            ikEngine,
            CarloLimbMotionPlanGenerator(ikEngine),
            DefaultLimbMotionPlanFollower(),
            seaArmLinks.mapIndexed { index, _ ->
                ServoJointAngleController(servos[index])
            }.toImmutableList(),
            NoopInertialStateEstimator
        )

        val base = DefaultKinematicBase(
            SimpleKinematicBaseId("carlo-arm-base"),
            immutableListOf(limb1),
            immutableMapOf(limb1Id to FrameTransformation.identity),
            NoopBodyController
        )

        limb1.setDesiredTaskSpaceTransform(
            limb1.links.map { it.dhParam }.toFrameTransformation(),
            BasicMotionConstraints(100, 10, 100, 100)
        )

        base.waitToStopMoving()

        device.disconnect().mapLeft { fail(it) }
    }

    @Test
    fun `test homing with fake arm`() {
        val limb1 = DefaultLimb(
            limb1Id,
            seaArmLinks,
            CarloFK(),
            ikEngine,
            CarloLimbMotionPlanGenerator(ikEngine),
            DefaultLimbMotionPlanFollower(),
            seaArmLinks.mapIndexed { index, _ ->
                SimulatedJointAngleController("$index")
            }.toImmutableList(),
            NoopInertialStateEstimator
        )

        val base = DefaultKinematicBase(
            SimpleKinematicBaseId("carlo-arm-base"),
            immutableListOf(limb1),
            immutableMapOf(limb1Id to FrameTransformation.identity),
            NoopBodyController
        )

        val target = FrameTransformation.fromTranslation(10, 0, 0)
        limb1.setDesiredTaskSpaceTransform(
            target,
            BasicMotionConstraints(100, 10, 100, 100)
        )

        base.waitToStopMoving()

        assertTrue(
            limb1.getCurrentTaskSpaceTransform().translation.isIdentical(
                target.translation, 1e-10
            )
        )
    }

    @Test
    fun `test blinking led`() {
        val injector = Script.makeScriptInjector().createChildInjector(Script.getDefaultModules())

        val device = getDevice(injector)

        val digitalOutFactory = injector.getInstance<UnprovisionedDigitalOutFactory>()
        val led1 = digitalOutFactory.makeUnprovisionedDigitalOut(
            device,
            DefaultAttachmentPoints.Pin(32)
        ).flatMap { device.add(it) }.getOrHandle { fail(it) }

        repeat(100) {
            val state = if (it % 2 == 0) DigitalState.HIGH else DigitalState.LOW
            led1.write(state)
            Thread.sleep(16)
        }

        device.disconnect().getOrHandle { fail(it) }
    }
}

private fun getDevice(injector: Injector) = binding<String, BowlerDevice> {
    val (device) = injector.getInstance<BowlerDeviceFactory>().makeBowlerDevice(
        DeviceId(
            DefaultDeviceTypes.Esp32wroom32,
            DefaultConnectionMethods.InternetAddress(
                InetAddress.getByAddress(
                    listOf(192, 168, 4, 1).map { it.toByte() }.toByteArray()
                )
            )
        )
    )
    device.connect()
    device
}.getOrHandle { fail(it) }
