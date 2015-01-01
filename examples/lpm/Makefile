# name of your application
APPLICATION = lpm_example

# If no BOARD is found in the environment, use this default:
BOARD ?= msbiot

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

# Define default pin mappings for some boards:
ifneq (,$(filter stm32f4discovery,$(BOARD)))
  export WKUP_GPIO ?= GPIO_0
endif
ifneq (,$(filter msbiot,$(BOARD)))
  export WKUP_GPIO ?= GPIO_8
endif

ifneq (,$(WKUP_GPIO))
  CFLAGS += -DWKUP_GPIO=$(WKUP_GPIO)
else
  # set random default
  CFLAGS += -DWKUP_GPIO=GPIO_0
endif

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
CFLAGS += -DDEVELHELP

BOARD_WHITELIST := msbiot stm32f4discovery

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

USEMODULE += vtimer

include $(RIOTBASE)/Makefile.include
