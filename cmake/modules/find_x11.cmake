find_package(X11 REQUIRED)

SET(X11_INCLUDE_DIRS "")
SET(X11_LIBRARIES "")
SET(X11_LIBRARY_DIRS "")

# Set up library and include paths
list(APPEND X11_INCLUDE_DIRS ${X11_X11_INCLUDE_PATH})
list(APPEND X11_LIBRARIES ${X11_X11_LIB})
list(APPEND X11_LIBRARIES ${RT_LIBRARY})

# Check for XRandR (modern resolution switching and gamma control)
if (NOT X11_Xrandr_FOUND)
    message(FATAL_ERROR "The RandR library and headers were not found")
endif()

list(APPEND X11_INCLUDE_DIRS ${X11_Xrandr_INCLUDE_PATH})
list(APPEND X11_LIBRARIES ${X11_Xrandr_LIB})

# Check for XInput (high-resolution cursor motion)
if (NOT X11_Xinput_FOUND)
    message(FATAL_ERROR "The XInput library and headers were not found")
endif()

list(APPEND X11_INCLUDE_DIRS ${X11_Xinput_INCLUDE_PATH})

if (X11_Xinput_LIB)
    list(APPEND X11_LIBRARIES ${X11_Xinput_LIB})
else()
    # Backwards compatibility (bug in CMake 2.8.7)
    list(APPEND X11_LIBRARIES Xi)
endif()

# Check for Xf86VidMode (fallback gamma control)
if (NOT X11_xf86vmode_FOUND)
    message(FATAL_ERROR "The Xf86VidMode library and headers were not found")
endif()

list(APPEND X11_INCLUDE_DIRS ${X11_xf86vmode_INCLUDE_PATH})

if (X11_Xxf86vm_LIB)
    list(APPEND X11_LIBRARIES ${X11_Xxf86vm_LIB})
else()
    # Backwards compatibility (see CMake bug 0006976)
    list(APPEND X11_LIBRARIES Xxf86vm)
endif()

# Check for Xkb (X keyboard extension)
if (NOT X11_Xkb_FOUND)
    message(FATAL_ERROR "The X keyboard extension headers were not found")
endif()

list(APPEND X11_INCLUDE_DIRS ${X11_Xkb_INCLUDE_PATH})
