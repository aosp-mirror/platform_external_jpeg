LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)
LOCAL_ARM_MODE := arm

LOCAL_SRC_FILES := \
    jcapimin.c jcapistd.c jccoefct.c jccolor.c jcdctmgr.c jchuff.c \
    jcinit.c jcmainct.c jcmarker.c jcmaster.c jcomapi.c jcparam.c \
    jcphuff.c jcprepct.c jcsample.c jctrans.c jdapimin.c jdapistd.c \
    jdatadst.c jdatasrc.c jdcoefct.c jdcolor.c jddctmgr.c jdhuff.c \
    jdinput.c jdmainct.c jdmarker.c jdmaster.c jdmerge.c jdphuff.c \
    jdpostct.c jdsample.c jdtrans.c jerror.c jfdctflt.c jfdctfst.c \
    jfdctint.c jidctflt.c jidctfst.c jidctint.c jidctred.c jquant1.c \
    jquant2.c jutils.c jmemmgr.c jmemnobs.c

LOCAL_SRC_FILES_arm += armv6_idct.S

LOCAL_CFLAGS += -DAVOID_TABLES
LOCAL_CFLAGS += -O3 -fstrict-aliasing -fprefetch-loop-arrays
LOCAL_CFLAGS += -Wno-unused-parameter

# enable tile based decode
LOCAL_CFLAGS += -DANDROID_TILE_BASED_DECODE

LOCAL_CFLAGS_x86 += -DANDROID_INTELSSE2_IDCT
LOCAL_SRC_FILES_x86 += jidctintelsse.c

LOCAL_SRC_FILES_arm64 += \
        jsimd_arm64_neon.S \
        jsimd_neon.c

ifeq ($(ARCH_ARM_HAVE_NEON),true)
  #use NEON accelerations
  LOCAL_CFLAGS_arm += -DNV_ARM_NEON -D__ARM_HAVE_NEON
  LOCAL_SRC_FILES_arm += \
      jsimd_arm_neon.S \
      jsimd_neon.c
else
  # enable armv6 idct assembly
  LOCAL_CFLAGS_arm += -DANDROID_ARMV6_IDCT
endif

# use mips assembler IDCT implementation if MIPS DSP-ASE is present
ifeq ($(strip $(ARCH_MIPS_HAS_DSP)),true)
LOCAL_CFLAGS_mips += -DANDROID_MIPS_IDCT
LOCAL_SRC_FILES_mips += \
    mips_jidctfst.c \
    mips_idct_le.S
endif

LOCAL_MODULE := libjpeg_static

LOCAL_EXPORT_C_INCLUDE_DIRS := $(LOCAL_PATH)

include $(BUILD_STATIC_LIBRARY)


# Build shared library
include $(CLEAR_VARS)

LOCAL_MODULE := libjpeg
LOCAL_WHOLE_STATIC_LIBRARIES = libjpeg_static

ifeq (,$(TARGET_BUILD_APPS))
LOCAL_SHARED_LIBRARIES := \
    libcutils
else
# unbundled branch, built against NDK.
LOCAL_SDK_VERSION := 17
endif

include $(BUILD_SHARED_LIBRARY)


# Build static library against NDK
include $(CLEAR_VARS)

LOCAL_ARM_MODE := arm

LOCAL_SRC_FILES := \
    jcapimin.c jcapistd.c jccoefct.c jccolor.c jcdctmgr.c jchuff.c \
    jcinit.c jcmainct.c jcmarker.c jcmaster.c jcomapi.c jcparam.c \
    jcphuff.c jcprepct.c jcsample.c jctrans.c jdapimin.c jdapistd.c \
    jdatadst.c jdatasrc.c jdcoefct.c jdcolor.c jddctmgr.c jdhuff.c \
    jdinput.c jdmainct.c jdmarker.c jdmaster.c jdmerge.c jdphuff.c \
    jdpostct.c jdsample.c jdtrans.c jerror.c jfdctflt.c jfdctfst.c \
    jfdctint.c jidctflt.c jidctfst.c jidctint.c jidctred.c jquant1.c \
    jquant2.c jutils.c jmemmgr.c jmemnobs.c

LOCAL_SRC_FILES_arm += armv6_idct.S

LOCAL_SDK_VERSION := 17
LOCAL_NDK_STL_VARIANT := none

LOCAL_CFLAGS += -DAVOID_TABLES
LOCAL_CFLAGS += -O3 -fstrict-aliasing -fprefetch-loop-arrays
LOCAL_CFLAGS += -Wno-unused-parameter
#LOCAL_CFLAGS += -march=armv6j

# enable tile based decode
LOCAL_CFLAGS += -DANDROID_TILE_BASED_DECODE

LOCAL_CFLAGS_x86 += -DANDROID_INTELSSE2_IDCT
LOCAL_SRC_FILES_x86 += jidctintelsse.c

LOCAL_SRC_FILES_arm64 += \
        jsimd_arm64_neon.S \
        jsimd_neon.c

ifeq ($(ARCH_ARM_HAVE_NEON),true)
  #use NEON accelerations
  LOCAL_CFLAGS_arm += -DNV_ARM_NEON -D__ARM_HAVE_NEON
  LOCAL_SRC_FILES_arm += \
      jsimd_arm_neon.S \
      jsimd_neon.c
else
  # enable armv6 idct assembly
  LOCAL_CFLAGS_arm += -DANDROID_ARMV6_IDCT
endif

# use mips assembler IDCT implementation if MIPS DSP-ASE is present
ifeq ($(strip $(ARCH_MIPS_HAS_DSP)),true)
LOCAL_CFLAGS_mips += -DANDROID_MIPS_IDCT
LOCAL_SRC_FILES_mips += \
    mips_jidctfst.c \
    mips_idct_le.S
endif

LOCAL_MODULE := libjpeg_static_ndk

LOCAL_EXPORT_C_INCLUDE_DIRS := $(LOCAL_PATH)

include $(BUILD_STATIC_LIBRARY)


# Build shared library against NDK
include $(CLEAR_VARS)

LOCAL_MODULE := libjpeg_ndk
LOCAL_WHOLE_STATIC_LIBRARIES = libjpeg_static_ndk
LOCAL_SDK_VERSION := 17
LOCAL_NDK_STL_VARIANT := none

include $(BUILD_SHARED_LIBRARY)


include $(CLEAR_VARS)
LOCAL_ARM_MODE := arm
LOCAL_SRC_FILES := \
	cjpeg.c rdswitch.c cdjpeg.c rdtarga.c rdppm.c rdgif.c rdbmp.c
LOCAL_MODULE:= cjpeg
LOCAL_MODULE_TAGS := eng
LOCAL_SHARED_LIBRARIES := libcutils libjpeg
include $(BUILD_EXECUTABLE)

include $(CLEAR_VARS)
LOCAL_ARM_MODE := arm
LOCAL_SRC_FILES := \
	djpeg.c cdjpeg.c wrppm.c wrgif.c wrbmp.c rdcolmap.c wrtarga.c
LOCAL_MODULE:= djpeg
LOCAL_MODULE_TAGS := eng
LOCAL_SHARED_LIBRARIES := libcutils libjpeg
include $(BUILD_EXECUTABLE)
