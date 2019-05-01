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

import com.neuronrobotics.bowlerkernel.hardware.Script
import org.jlleitschuh.guice.getInstance
import org.junit.jupiter.api.Test
import org.octogonapus.ktguava.collections.emptyImmutableList

internal class CarloArmTest {

    @Test
    fun `test running the arm`() {
        val script = Script.makeScriptInjector().createChildInjector(
            Script.getDefaultModules()
        ).getInstance<CarloArm>()

        val result = script.startScript(emptyImmutableList())
        Thread.sleep(1000)
        script.stopAndCleanUp()
        println(result)
    }
}
