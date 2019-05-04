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

import com.neuronrobotics.bowlerkernel.kinematics.limb.Limb
import com.neuronrobotics.bowlerkernel.kinematics.motion.BasicMotionConstraints
import com.neuronrobotics.bowlerkernel.kinematics.motion.FrameTransformation
import com.neuronrobotics.bowlerkernel.kinematics.motion.InverseKinematicsSolver
import com.neuronrobotics.bowlerkernel.kinematics.motion.MotionConstraints
import com.neuronrobotics.bowlerkernel.kinematics.motion.plan.LimbMotionPlan
import com.neuronrobotics.bowlerkernel.kinematics.motion.plan.LimbMotionPlanGenerator
import com.neuronrobotics.bowlerkernel.kinematics.motion.plan.LimbMotionPlanStep
import org.octogonapus.ktguava.collections.toImmutableList

class CarloLimbMotionPlanGenerator(
    private val ikSolver: InverseKinematicsSolver,
    private val numStepsPerPlan: Int = 5
) : LimbMotionPlanGenerator {

    override fun generatePlanForTaskSpaceTransform(
        limb: Limb,
        currentTaskSpaceTransform: FrameTransformation,
        targetTaskSpaceTransform: FrameTransformation,
        motionConstraints: MotionConstraints
    ): LimbMotionPlan {
        val currentAngles = limb.getCurrentJointAngles()

        val targetAngles = ikSolver.solveChain(
            limb.getCurrentJointAngles(),
            targetTaskSpaceTransform
        )

        val steps = (1..numStepsPerPlan).map { stepNum ->
            val fraction = stepNum / numStepsPerPlan.toDouble()

            val angles = targetAngles.mapIndexed { index, elem ->
                currentAngles[index] + (elem - currentAngles[index]) * fraction
            }.toImmutableList()

            LimbMotionPlanStep(
                angles,
                BasicMotionConstraints(
                    motionConstraints.motionDuration / numStepsPerPlan,
                    motionConstraints.maximumVelocity,
                    motionConstraints.maximumAcceleration,
                    motionConstraints.maximumJerk
                )
            )
        }.toImmutableList()

        return LimbMotionPlan(steps)
    }
}
