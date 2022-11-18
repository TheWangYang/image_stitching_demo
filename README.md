# cpp_test_demo

## Environment
1. Windows10;
2. OpenCV, version: 4.5.3;
3. Compiler: gcc, version: 8.1.0;
4. CMake version: 3.24.0-rc5; 
5. Visual Studio Code(VSCode) version: 2022;

## Introduction
This repository was created for me when I started cpp development. It mainly records the projects and implementation codes I have done in cpp development.

## Projects List
### [Image Stitch] a computer vision project based on OpenCV(version: 4.5.3)
#### Function Blocks
1. SURF local description operator is implemented to extract features of the picture and then registration and stitching (picture stitching, fusion field);
2. Add functions: Batch compile.cpp source files using the CMake command to generate multiple executable.exe files;
3. todo...
#### Configurations
* Add .vscode dir into this project root directory before run it;
* Add c_cpp_properties.json file into .vscode fir, and its content: 
```
{
    "configurations": [
        {
            "name": "Win32",
            "includePath": [
                "${workspaceFolder}/**",
                "F:/cpp_tools/tools/opencv_4_build/install/include",
                "F:/cpp_tools/tools/opencv_4_build/install/include/opencv2",
                "F:/cpp_tools/tools/mingw64/lib/gcc/x86_64-w64-mingw32/8.1.0/include",
                "F:/cpp_tools/tools/mingw64/lib/gcc/x86_64-w64-mingw32/8.1.0/include/c++",
                "F:/cpp_tools/tools/mingw64/lib/gcc/x86_64-w64-mingw32/8.1.0/include/c++/x86_64-w64-mingw32",
                "F:/cpp_tools/tools/mingw64/lib/gcc/x86_64-w64-mingw32/8.1.0/include/c++/x86_64-w64-mingw32/32"
                
            ],
            "defines": [],
            "compilerPath": "F:/cpp_tools/tools/mingw64/bin/gcc.exe",
            "cStandard": "c11",
            "cppStandard": "c++17",
            "intelliSenseMode": "clang-x64"
        }
    ],
    "version": 4
}
```
* Add the launch.json file into build dir, its content:
```
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "opencv4.5.3 debuge",
            "type": "cppdbg",
            "request": "launch",
            "program": "${fileDirname}\\${fileBasenameNoExtension}.exe",
            "args": [],
            "stopAtEntry": false,
            "cwd": "${workspaceFolder}",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb",
            "miDebuggerPath": "F:/cpp_tools/tools/mingw64/bin/gdb.exe",
            "setupCommands": [
                {
                    "description": "为 gdb 启用整齐打印",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": false
                }
            ],
            "preLaunchTask": "opencv4.5.3 compile task"
        }
    ]
}

```
* Add the tasks.json file into build dir, its content:
```
{
    "version": "2.0.0",
    "tasks": [
        {
            "type": "shell",
            "label": "opencv4.5.3 compile task",
            "command": "F:/cpp_tools/tools/mingw64/bin/g++.exe",
            "args": [
                "-fdiagnostics-color=always",
                "-g",
                "${file}",
                "-o",
                "${fileDirname}\\${fileBasenameNoExtension}.exe",
                "-I",
                "F:\\cpp_tools\\tools\\opencv_4_build\\install\\include",
                "-I",
                "F:\\cpp_tools\\tools\\opencv_4_build\\install\\include\\opencv2",
                "-L",
                "F:\\cpp_tools\\tools\\opencv_4_build\\bin",
                "-l",
                "libopencv_calib3d453",
                "-l",
                "libopencv_core453",
                "-l",
                "libopencv_dnn453",
                "-l",
                "libopencv_features2d453",
                "-l",
                "libopencv_flann453",
                "-l",
                "libopencv_highgui453",
                "-l",
                "libopencv_imgcodecs453",
                "-l",
                "libopencv_imgproc453",
                "-l",
                "libopencv_ml453",
                "-l",
                "libopencv_objdetect453",
                "-l",
                "libopencv_photo453",
                "-l",
                "libopencv_stitching453",
                "-l",
                "libopencv_video453",
                "-l",
                "libopencv_videoio453"
            ],
            "options": {
                "cwd": "F:/cpp_tools/tools/mingw64/bin"
            },
            "problemMatcher": [
                "$gcc"
            ],
            "group": "build"
        },
    ]
}
```
* If you use Code Runner extension to run code, then the settings.json you will need:
```
{
    "C_Cpp.errorSquiggles": "Disabled",
    // "code-runner.executorMap": {
        
    //     "c": "cd $dir && gcc '$fileName' -o '$fileNameWithoutExt.exe' -Wall -g -O2 -static-libgcc -std=c11 -fexec-charset=UTF-8 && &'$dir$fileNameWithoutExt'",
    //just change the following paths with using your personal environment paths
    //     "cpp": "cd $dir && g++ $fileName -o $fileNameWithoutExt -I F:/cpp_tools/tools/opencv_4_build/install/include -I F:/cpp_tools/tools/opencv_4_build/install/include/opencv2 -L F:/cpp_tools/tools/opencv_4_build/install/x64/mingw/lib -l libopencv_world453 && $dir$fileNameWithoutExt "
    // },
    "files.associations": {
        "array": "cpp",
        "atomic": "cpp",
        "*.tcc": "cpp",
        "cctype": "cpp",
        "chrono": "cpp",
        "clocale": "cpp",
        "cmath": "cpp",
        "complex": "cpp",
        "condition_variable": "cpp",
        "cstdarg": "cpp",
        "cstddef": "cpp",
        "cstdint": "cpp",
        "cstdio": "cpp",
        "cstdlib": "cpp",
        "cstring": "cpp",
        "ctime": "cpp",
        "cwchar": "cpp",
        "cwctype": "cpp",
        "deque": "cpp",
        "unordered_map": "cpp",
        "vector": "cpp",
        "exception": "cpp",
        "algorithm": "cpp",
        "functional": "cpp",
        "iterator": "cpp",
        "memory": "cpp",
        "memory_resource": "cpp",
        "optional": "cpp",
        "ratio": "cpp",
        "string": "cpp",
        "string_view": "cpp",
        "system_error": "cpp",
        "tuple": "cpp",
        "type_traits": "cpp",
        "utility": "cpp",
        "fstream": "cpp",
        "initializer_list": "cpp",
        "iosfwd": "cpp",
        "iostream": "cpp",
        "istream": "cpp",
        "limits": "cpp",
        "mutex": "cpp",
        "new": "cpp",
        "ostream": "cpp",
        "sstream": "cpp",
        "stdexcept": "cpp",
        "streambuf": "cpp",
        "thread": "cpp",
        "typeinfo": "cpp"
    }

}

```
* Some additional configuration details will be added selectively based on your feedback...

#### How to run?
1. git clone https://gitee.com/SCKDKT/cpp_test_demo.git;
2. open terminal and type: mkdir build->cd build->cmake .. -G "MinGW Makefiles"->mingw32-make;
3. then, you will find some .exe files in build directory;
4. just click double twice on each of the .exe files;
5. then, the results will display just a few seconds later;

### Other projects
#### Function Blocks
1. todo...
#### Configurations
* todo...
#### About
1. todo...

## Contributors

1. [TheWangYang](https://gitee.com/SCKDKT)


## References
1. [OpenCVDemoCpp](https://github.com/Vaccae/OpenCVDemoCpp)


