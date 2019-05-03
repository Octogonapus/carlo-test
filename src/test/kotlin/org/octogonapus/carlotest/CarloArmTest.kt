package org.octogonapus.carlotest

import com.google.common.collect.ImmutableList
import com.neuronrobotics.bowlerkernel.kinematics.base.DefaultKinematicBase
import com.neuronrobotics.bowlerkernel.kinematics.base.baseid.SimpleKinematicBaseId
import com.neuronrobotics.bowlerkernel.kinematics.closedloop.NoopBodyController
import com.neuronrobotics.bowlerkernel.kinematics.limb.DefaultLimb
import com.neuronrobotics.bowlerkernel.kinematics.limb.limbid.SimpleLimbId
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.DefaultLink
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.DhParam
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.Link
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.LinkType
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.toFrameTransformation
import com.neuronrobotics.bowlerkernel.kinematics.motion.ForwardKinematicsSolver
import com.neuronrobotics.bowlerkernel.kinematics.motion.FrameTransformation
import com.neuronrobotics.bowlerkernel.kinematics.motion.InverseKinematicsSolver
import com.neuronrobotics.bowlerkernel.kinematics.motion.NoopInertialStateEstimator
import com.neuronrobotics.bowlerkernel.kinematics.motion.plan.DefaultLimbMotionPlanFollower
import com.neuronrobotics.bowlerkernel.util.Limits
import org.junit.jupiter.api.Test
import org.octogonapus.ktguava.collections.immutableListOf
import org.octogonapus.ktguava.collections.immutableMapOf
import org.octogonapus.ktguava.collections.toImmutableList

internal class CarloArmTest {

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

    @Test
    fun `test homing with real arm`() {
        val limb1 = DefaultLimb(
            limb1Id,
            links,
            fkEngine,
            ikEngine,
            CarloLimbMotionPlanGenerator(ikEngine),
            DefaultLimbMotionPlanFollower(),
            links.mapIndexed { index, _ ->
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

        val carlo = CarloArm(base)
        carlo.homeLimbs(200)
    }
}
