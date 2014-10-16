# Define target name
set (TARGET_NAME DetourUtils)

# Define source files
file (GLOB CPP_FILES source/*.cpp)
file (GLOB H_FILES include/*.h)
set (SOURCE_FILES ${CPP_FILES} ${H_FILES})

# Define dependency libs
set (INCLUDE_DIRS_ONLY include ../Detour/include ../Recast/include ../DetourTileCache/include ../DetourCrowd/include)


# Setup target
setup_library ()
