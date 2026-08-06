# Required platform files.
PLATFORMSRC := $(CHIBIOS)/os/hal/ports/TI/AM67/hal_lld.c \
               $(CHIBIOS)/os/hal/ports/TI/AM67/am67_vim.c \
               $(CHIBIOS)/os/hal/ports/TI/AM67/hal_st_lld.c \
               $(CHIBIOS)/os/hal/ports/TI/AM67/hal_serial_lld.c \
               $(CHIBIOS)/os/hal/ports/TI/AM67/hal_spi_lld.c \
               $(CHIBIOS)/os/hal/ports/TI/AM67/hal_i2c_lld.c \
               $(CHIBIOS)/os/hal/ports/TI/AM67/am67_epwm.c \
               $(CHIBIOS)/os/hal/ports/TI/AM67/am67_ecap.c \
               $(CHIBIOS)/os/hal/ports/TI/AM67/am67_mailbox.c

# Required include directories.
PLATFORMINC := $(CHIBIOS)/os/hal/ports/TI/AM67

# Shared variables.
ALLCSRC += $(PLATFORMSRC)
ALLINC  += $(PLATFORMINC)
