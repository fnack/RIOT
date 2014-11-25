# name of your application
APPLICATION = nhdp

# If no BOARD is found in the environment, use this default:
BOARD ?= native

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

USEPKG += oonf_api

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
CFLAGS += -DDEVELHELP
CFLAGS += -DRIOT

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

USEMODULE += defaulttransceiver
USEMODULE += netdev_base
USEMODULE += netapi
USEMODULE += nomac
USEMODULE += nhdp
USEMODULE += vtimer
USEMODULE += net_help

export INCLUDES += -I$(RIOTBASE)/sys/net/include \
                 -I$(RIOTBASE)/pkg/oonf_api/oonf_api/src-api/

include $(RIOTBASE)/Makefile.include
