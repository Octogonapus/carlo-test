buildCache {
    local(DirectoryBuildCache::class.java) {
        isEnabled = true
        directory = file("${rootDir.path}/build-cache")
    }
}

rootProject.name = "carlo-test"
