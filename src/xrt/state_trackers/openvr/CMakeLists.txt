# Copyright 2026, Beyley Cardellio
# Copyright 2026, NVIDIA CORPORATION.
# SPDX-License-Identifier: BSL-1.0

add_library(
	st_openvr STATIC
	openvr_devices.cpp
	openvr_devices.hpp
	openvr_events.cpp
	openvr_events.hpp
	openvr_negotiate.cpp
	common/openvr_error.cpp
	common/openvr_error.hpp
	common/openvr_logger.cpp
	common/openvr_logger.hpp
	common/openvr_math.cpp
	common/openvr_math.hpp
	compositor/openvr_compositor.cpp
	compositor/openvr_compositor.hpp
	interfaces/XRTVRApplications.cpp
	interfaces/XRTVRApplications.hpp
	interfaces/XRTVRChaperone.cpp
	interfaces/XRTVRChaperone.hpp
	interfaces/XRTVRChaperoneSetup.cpp
	interfaces/XRTVRChaperoneSetup.hpp
	interfaces/XRTVRClientCore.cpp
	interfaces/XRTVRClientCore.hpp
	interfaces/XRTVRCompositor.cpp
	interfaces/XRTVRCompositor.hpp
	interfaces/XRTVRControlPanel.cpp
	interfaces/XRTVRControlPanel.hpp
	interfaces/XRTVRDebug.cpp
	interfaces/XRTVRDebug.hpp
	interfaces/XRTVRDriverManager.cpp
	interfaces/XRTVRDriverManager.hpp
	interfaces/XRTVRExtendedDisplay.cpp
	interfaces/XRTVRExtendedDisplay.hpp
	interfaces/XRTVRHeadsetView.cpp
	interfaces/XRTVRHeadsetView.hpp
	interfaces/XRTVRIOBuffer.cpp
	interfaces/XRTVRIOBuffer.hpp
	interfaces/XRTVRIPCResourceManagerClient.cpp
	interfaces/XRTVRIPCResourceManagerClient.hpp
	interfaces/XRTVRInput.cpp
	interfaces/XRTVRInput.hpp
	interfaces/XRTVRNotifications.cpp
	interfaces/XRTVRNotifications.hpp
	interfaces/XRTVROverlay.cpp
	interfaces/XRTVROverlay.hpp
	interfaces/XRTVROverlayView.cpp
	interfaces/XRTVROverlayView.hpp
	interfaces/XRTVRRenderModels.cpp
	interfaces/XRTVRRenderModels.hpp
	interfaces/XRTVRResources.cpp
	interfaces/XRTVRResources.hpp
	interfaces/XRTVRScreenshots.cpp
	interfaces/XRTVRScreenshots.hpp
	interfaces/XRTVRSettings.cpp
	interfaces/XRTVRSettings.hpp
	interfaces/XRTVRSpatialAnchors.cpp
	interfaces/XRTVRSpatialAnchors.hpp
	interfaces/XRTVRSystem.cpp
	interfaces/XRTVRSystem.hpp
	interfaces/XRTVRTrackedCamera.cpp
	interfaces/XRTVRTrackedCamera.hpp
	interfaces/old/IVRChaperone/XRTVRChaperone_003.hpp
	interfaces/old/IVRChaperoneSetup/XRTVRChaperoneSetup_005.hpp
	interfaces/old/IVRCompositor/XRTVRCompositor_022.hpp
	interfaces/old/IVRCompositor/XRTVRCompositor_024.hpp
	interfaces/old/IVRCompositor/XRTVRCompositor_026.hpp
	interfaces/old/IVRCompositor/XRTVRCompositor_027.hpp
	interfaces/old/IVRCompositor/XRTVRCompositor_028.hpp
	interfaces/old/IVRSystem/XRTVRSystem_019.hpp
	interfaces/old/IVRSystem/XRTVRSystem_020.hpp
	interfaces/old/IVRSystem/XRTVRSystem_021.hpp
	interfaces/old/IVRSystem/XRTVRSystem_023.hpp
	)

if(XRT_HAVE_VULKAN)
	target_sources(st_openvr PRIVATE compositor/openvr_compositor_vulkan.cpp)

	target_link_libraries(st_openvr PRIVATE aux_vk aux_render aux_shaders aux_render_cache)
endif()

target_link_libraries(
	st_openvr
	PRIVATE xrt-interfaces xrt-external-openvr
	PUBLIC aux_os aux_glue comp_client
	)

target_include_directories(st_openvr PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/)

target_include_directories(
	st_openvr
	PRIVATE ${CMAKE_CURRENT_BINARY_DIR}
	INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/..
	)
