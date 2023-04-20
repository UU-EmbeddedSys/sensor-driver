# Sensor Driver
Implementation of the sensor driver using Zephyr. The project uses modules.

## How to create a new module
Let's look how we created the drivers folder.<br>
- create the folder <module>
- inside the module folder create the `zephyr` folder (should be something like this my_module/zephyr)
- inside the zephyr folder create `module.yml`
```yml
build:
  cmake: zephyr
  kconfig: zephyr/Kconfig
```
- If any configuration are present, create the `Kconfig` file
- In order to be linked by zephyr use these in the CMakeFile
```cmake
  zephyr_include_directories(.)
  zephyr_library()
  zephyr_library_sources(your_source.c)
```
- Enable the module can be done manually by adding it to the `prj.conf`
```
CONFIG_MY_MODULE=y
```
- In your top CMakeFile.txt append your module to the Zephyr ones
```
  list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/my_module )
```


## Debug Configuration
In the .vscode folder is present a `launch.json` file. This should work as it is, if not change the gdb that may have a different name.
### OpenOCD
Before launching the debugger run OpenOCD in your terminal
```
openocd -f interface/picoprobe.cfg -f target/rp2040.cfg -s tcl
```