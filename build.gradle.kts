import com.adarshr.gradle.testlogger.theme.ThemeType
import com.github.spotbugs.SpotBugsTask
import org.gradle.api.tasks.testing.logging.TestExceptionFormat
import org.gradle.api.tasks.testing.logging.TestLogEvent
import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

plugins {
    jacoco
    kotlin("jvm") version "1.3.31"
    id("com.diffplug.gradle.spotless") version "3.22.0"
    id("org.jlleitschuh.gradle.ktlint") version "7.3.0"
    id("com.github.spotbugs") version "1.7.1"
    id("io.gitlab.arturbosch.detekt") version "1.0.0-RC12"
    id("org.jetbrains.dokka") version "0.9.18"
    id("com.adarshr.test-logger") version "1.6.0"
}

group = "org.octogonapus"
version = "0.1.0"

val ktlintVersion = "0.29.0"
val kotlinVersion = "1.3.31"
val junitJupiterVersion = "5.4.0"
val ktUnitsVersion = "0.1.0"

val spotlessLicenseHeaderDelimiter = "(@|package|import)"

apply {
    plugin("com.diffplug.gradle.spotless")
    plugin("com.adarshr.test-logger")
    plugin("kotlin")
    plugin("org.jlleitschuh.gradle.ktlint")
    plugin("checkstyle")
    plugin("jacoco")
    plugin("com.github.spotbugs")
    plugin("io.gitlab.arturbosch.detekt")
    plugin("org.jetbrains.dokka")
}

repositories {
    mavenCentral()
    jcenter()
    maven("https://dl.bintray.com/arrow-kt/arrow-kt/")
    maven("https://oss.jfrog.org/artifactory/oss-snapshot-local/")
    maven("https://dl.bintray.com/commonwealthrobotics/maven-artifacts")
    mavenLocal()
}

fun DependencyHandler.junitJupiter(name: String) =
    create(group = "org.junit.jupiter", name = name, version = junitJupiterVersion)

fun DependencyHandler.arrow(name: String) =
    create(group = "io.arrow-kt", name = name, version = "0.9.0")

dependencies {
    implementation(kotlin("stdlib-jdk8", kotlinVersion))
    implementation("com.neuronrobotics:bowler-kernel-kinematics:0.0.26")
    // implementation("com.neuronrobotics:bowler-kinematics-core:0.0.16")
    implementation(
        group = "org.octogonapus",
        name = "kt-guava-core",
        version = "0.2.1"
    )
    implementation(
        group = "com.google.guava",
        name = "guava",
        version = "27.0.1-jre"
    )
    implementation(
        group = "com.google.inject",
        name = "guice",
        version = "4.1.0"
    )
    implementation(
        group = "com.google.inject.extensions",
        name = "guice-assistedinject",
        version = "4.1.0"
    )
    implementation(
        group = "org.jlleitschuh.guice",
        name = "kotlin-guiced-core",
        version = "0.0.5"
    )

    implementation(arrow("arrow-core-data"))
    implementation(arrow("arrow-core-extensions"))
    implementation(arrow("arrow-syntax"))
    implementation(arrow("arrow-typeclasses"))
    implementation(arrow("arrow-extras-data"))
    implementation(arrow("arrow-extras-extensions"))

    testCompile(junitJupiter("junit-jupiter"))
    testRuntime(
        group = "org.junit.platform",
        name = "junit-platform-launcher",
        version = "1.0.0"
    )
}

// Configures the Jacoco tool version to be the same for all projects that have it applied.
pluginManager.withPlugin("jacoco") {
    // If this project has the plugin applied, configure the tool version.
    jacoco {
        toolVersion = "0.8.3"
    }
}

tasks.withType<Test> {
    testLogging {
        showStandardStreams = true
        exceptionFormat = TestExceptionFormat.FULL
    }
}

testlogger {
    theme = ThemeType.STANDARD_PARALLEL
}

spotless {
    /*
     * We use spotless to lint the Gradle Kotlin DSL files that make up the build.
     * These checks are dependencies of the `check` task.
     */
    kotlinGradle {
        ktlint(ktlintVersion)
        trimTrailingWhitespace()
    }
    freshmark {
        trimTrailingWhitespace()
        indentWithSpaces(2)
        endWithNewline()
    }
    format("extraneous") {
        target("src/**/*.fxml")
        trimTrailingWhitespace()
        indentWithSpaces(2)
        endWithNewline()
    }
    kotlin {
        ktlint(ktlintVersion)
        trimTrailingWhitespace()
        indentWithSpaces(2)
        endWithNewline()
        @Suppress("INACCESSIBLE_TYPE")
        licenseHeaderFile(
            "${rootProject.rootDir}/config/spotless/carlo-test.license",
            spotlessLicenseHeaderDelimiter
        )
    }
}

tasks.withType<KotlinCompile> {
    kotlinOptions.jvmTarget = "1.8"
}

tasks.withType<Test> {
    @Suppress("UnstableApiUsage")
    useJUnitPlatform {
        filter {
            includeTestsMatching("*Test")
            includeTestsMatching("*Tests")
            includeTestsMatching("*Spec")
        }

        /*
         * Performance tests are only really run during development.
         * They don't need to run in CI or as part of regular development.
         */
        excludeTags("performance")

        /*
         * Marking a test as `slow` will excluded it from being run as part of the regular CI system.
         */
        excludeTags("slow")
    }

    if (project.hasProperty("jenkinsBuild") || project.hasProperty("headless")) {
        jvmArgs = listOf(
            "-Djava.awt.headless=true",
            "-Dtestfx.robot=glass",
            "-Dtestfx.headless=true",
            "-Dprism.order=sw",
            "-Dprism.text=t2k"
        )
    }

    testLogging {
        events(
            TestLogEvent.FAILED,
            TestLogEvent.PASSED,
            TestLogEvent.SKIPPED,
            TestLogEvent.STARTED
        )
        displayGranularity = 0
        showExceptions = true
        showCauses = true
        showStackTraces = true
        exceptionFormat = TestExceptionFormat.FULL
    }

    @Suppress("UnstableApiUsage")
    reports.junitXml.destination = file("${rootProject.buildDir}/test-results/${project.name}")
}

tasks.withType<JacocoReport> {
    @Suppress("UnstableApiUsage")
    reports {
        html.isEnabled = true
        xml.isEnabled = true
        csv.isEnabled = false
    }
}

detekt {
    toolVersion = "1.0.0-RC12"
    input = files(
        "src/main/kotlin",
        "src/test/kotlin"
    )
    parallel = true
    config = files("${rootProject.rootDir}/config/detekt/config.yml")
}

checkstyle {
    toolVersion = "8.1"
}

spotbugs {
    toolVersion = "4.0.0-beta1"
    excludeFilter = file("${rootProject.rootDir}/config/spotbugs/spotbugs-excludeFilter.xml")
}

tasks.withType<SpotBugsTask> {
    @Suppress("UnstableApiUsage")
    reports {
        xml.isEnabled = false
        emacs.isEnabled = false
        html.isEnabled = true
    }
}

tasks.wrapper {
    gradleVersion = "5.4"
    distributionType = Wrapper.DistributionType.ALL
}

/**
 * Configures the [checkstyle][org.gradle.api.plugins.quality.CheckstyleExtension] project extension.
 */
fun Project.`checkstyle`(configure: org.gradle.api.plugins.quality.CheckstyleExtension.() -> Unit) =
    extensions.configure("checkstyle", configure)
