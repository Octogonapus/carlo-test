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

import com.neuronrobotics.bowlerkernel.kinematics.motion.FrameTransformation
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import org.octogonapus.ktguava.collections.toImmutableList

internal class CarloIKTest {

    private val fk = CarloFK()
    private val ik = CarloIK()

    @Test
    fun `test 3dof ik`() {
        // TODO: This break with numbers less than zero and with zero
        (-30..30 step 1).map { targetPos ->
            val target = FrameTransformation.fromTranslation(targetPos, 0, 0)
            val resultAngles = ik.solveChain(
                seaArmLinks,
                seaArmLinks.map { 0.0 }.toImmutableList(),
                target
            )
            val resultTarget = fk.solveChain(seaArmLinks, resultAngles)
            assertTrue(target.translation.isIdentical(resultTarget.translation, 1e-10)) {
                """
                Target:
                $target
                Result:
                $resultTarget
                Result angles:
                $resultAngles
                """.trimIndent()
            }
        }
    }
}
