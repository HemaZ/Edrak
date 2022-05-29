from conans import ConanFile, CMake, tools
from importlib_metadata import requires


class EdrakConan(ConanFile):
    name = "Edrak"
    version = "0.1"
    license = "MIT"
    author = "Ibrahim Essam Abdelmonem ibra.es95@gmail.com"
    url = "<Package recipe repository url here, for issues about the package>"
    description = "A toolbox for autonomous perception"
    topics = ("Computer Vision", "SLAM", "Robotics")
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": False, "fPIC": True}
    generators = "cmake"
    requires = "boost/1.79.0", "opencv/4.5.5", "eigen/3.3.9", "sophus/22.04.1", "ceres-solver/2.0.0", "catch2/2.13.9", "imgui/1.87", "glfw/3.3.7", "glew/2.2.0", "fmt/8.1.1"

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    # def source(self):
    #     self.run(
    #         "git clone https://github.com/HemaZ/Edrak.git --recurse-submodules")
    #     # This small hack might be useful to guarantee proper /MT /MD linkage
    #     # in MSVC if the packaged project doesn't have variables to set it
    #     # properly
    #     tools.replace_in_file("Edrak/CMakeLists.txt", "project(Edrak)",
    #                           '''PROJECT(Edrak)
    #                             include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
    #                             conan_basic_setup()''')

    def build(self):
        cmake = CMake(self)
        cmake.configure(source_folder="Edrak")
        cmake.build()

        # Explicit way:
        # self.run('cmake %s/hello %s'
        #          % (self.source_folder, cmake.command_line))
        # self.run("cmake --build . %s" % cmake.build_config)

    def package(self):
        self.copy("*", dst="include", src="Edrak/include")
        self.copy("*edrak.lib", dst="lib", keep_path=False)
        self.copy("*.dll", dst="bin", keep_path=False)
        self.copy("*.so", dst="lib", keep_path=False)
        self.copy("*.dylib", dst="lib", keep_path=False)
        self.copy("*.a", dst="lib", keep_path=False)

    def package_info(self):
        self.cpp_info.libs = ["Edrak"]
