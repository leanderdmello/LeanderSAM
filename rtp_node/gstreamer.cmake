# Copyright 2025 Avular Holding B.V.
# All Rights Reserved
# You may use this code under the terms of the Avular
# Software End-User License Agreement.
#
# You should have received a copy of the Avular
# Software End-User License Agreement license with
# this file, or download it from: avular.com/eula

# Find GStreamer libraries and headers

find_package(PkgConfig REQUIRED)
pkg_check_modules(gstreamer-1.0 IMPORTED_TARGET gstreamer-1.0)
pkg_check_modules(gstapp-1.0 IMPORTED_TARGET gstreamer-app-1.0)
pkg_check_modules(gstreamer-rtsp-1.0 IMPORTED_TARGET gstreamer-rtsp-1.0)


if(TARGET PkgConfig::gstreamer-1.0 AND
   TARGET PkgConfig::gstapp-1.0 AND
   TARGET PkgConfig::gstreamer-rtsp-1.0)

	add_library(gstreamer INTERFACE IMPORTED GLOBAL)
	target_link_libraries(gstreamer INTERFACE
		PkgConfig::gstreamer-1.0
		PkgConfig::gstapp-1.0
		PkgConfig::gstreamer-rtsp-1.0)
endif()

if(TARGET gstreamer)
    message(STATUS "Found GStreamer on this system")
else()
    message("DEPENDENCY WARNING GStreamer is not installed on your system.\n
    See https://gstreamer.freedesktop.org/documentation/installing/on-linux.html?gi-language=c\n
	aditionally, install libgstrtspserver-1.0-dev\n
	List installed packaged with pkg-config --list-all | grep -i gstreamer | sort")
    return()
endif()
