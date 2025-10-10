ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_USBH TRUE,$(HALCONF)),)
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/STM32/LLD/USBHv1/hal_usbh_lld.c
endif
else
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/STM32/LLD/USBHv1/hal_usbh_lld.c
endif

PLATFORMINC += $(CHIBIOS)/os/hal/ports/STM32/LLD/USBHv1
