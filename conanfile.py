from conan import ConanFile
from conan.tools.cmake import cmake_layout


class BehaviourTreesExploration(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "CMakeToolchain"

    def requirements(self):
        self.requires("behaviortree.cpp/4.7.2")
        self.requires("flexiv_rdk/1.8")
        self.requires("eigen/3.4.0")
        self.requires("cynlr_camera/1.0.0")
        self.requires("cynlr_gripper/2.0.0")
        self.requires("tl-expected/1.2.0")
        self.requires("spdlog/1.14.0", force=True)
        self.requires("tinyxml2/10.0.0", force=True)
        self.requires("fmt/10.1.1", override=True)

    def configure(self):
        # pcre2grep tool fails to link on Windows (missing zlib/bzip2 in link step)
        # We only need the pcre2 library, not the grep tool
        self.options["pcre2"].build_pcre2grep = False

    def layout(self):
        cmake_layout(self)
