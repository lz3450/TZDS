################################################################
# config
################################################################
include config.mk

################################################################
# output directories
################################################################
depdir = .dep
irdir = .ir
asmdir = .asm
objdir = .obj
dbdir = .db
targetdir = output

################################################################
# toolchain
################################################################
LLVM_DIR = ../../toolchain
LLVM_SYSROOT = $(LLVM_DIR)/lib/clang-runtimes/armv8m.main_hard_cortex-m33_fpv5-sp-d16

GNU_TOOLCHAIN_DIR = ../../toolchain
GNU_TOOLCHAIN_PREFIX = $(GNU_TOOLCHAIN_DIR)/bin/arm-none-eabi
GNU_SYSROOT = $(GNU_TOOLCHAIN_DIR)/arm-none-eabi

COMPILER ?= clang

CC =
AS =
LD =
OBJCOPY =
OBJDUMP =
READELF =

ifeq ($(COMPILER),clang)
# LLVM
CC = $(LLVM_DIR)/bin/clang
AS = $(LLVM_DIR)/bin/clang
LLVMLINK = $(LLVM_DIR)/bin/llvm-link
LLVMOPT = $(LLVM_DIR)/bin/opt
# LD = $(LLVM_DIR)/bin/clang
LD = $(GNU_TOOLCHAIN_PREFIX)-gcc
# OBJCOPY = $(LLVM_DIR)/bin/llvm-objcopy
# OBJDUMP = $(LLVM_DIR)/bin/llvm-objdump
# READELF = $(LLVM_DIR)/bin/llvm-readelf
OBJCOPY = $(GNU_TOOLCHAIN_PREFIX)-objcopy
OBJDUMP = $(GNU_TOOLCHAIN_PREFIX)-objdump
READELF = $(GNU_TOOLCHAIN_PREFIX)-readelf
else ifeq ($(COMPILER),gcc)
# GNU
CC = $(GNU_TOOLCHAIN_PREFIX)-gcc
AS = $(GNU_TOOLCHAIN_PREFIX)-gcc
LD = $(GNU_TOOLCHAIN_PREFIX)-gcc
OBJCOPY = $(GNU_TOOLCHAIN_PREFIX)-objcopy
OBJDUMP = $(GNU_TOOLCHAIN_PREFIX)-objdump
READELF = $(GNU_TOOLCHAIN_PREFIX)-readelf
endif

$(if $(CC),,$(error Unknow compiler!))

# JLink
JLINK_EXE = ../../jlink/JLinkExe

################################################################
# OPTIMIZTION_FLAGS
################################################################
DEBUG_FLAGS = -DDEBUG -g -O0
RELEASE_FLAGS = -DRELEASE -O2

OPTIMIZTION_FLAGS =
ifeq ($(build),debug)
OPTIMIZTION_FLAGS += $(DEBUG_FLAGS)
else ifeq ($(build),release)
OPTIMIZTION_FLAGS += $(RELEASE_FLAGS)
else

endif

$(if $(OPTIMIZTION_FLAGS),,$(error Unknow build configuration!))

################################################################
# target flag
################################################################
TARGET_FLAG =
ifeq ($(COMPILER),clang)
TARGET_FLAG += --target=thumbv8m.main-none-unknown-eabihf
endif

################################################################
# ARCHFLAGS
################################################################
ARCHFLAGS =
ARCHFLAGS += -mcpu=cortex-m33
# ARCHFLAGS += -march=armv8-m.main
# ARCHFLAGS += -mtune=cortex-m33
ARCHFLAGS += -mfloat-abi=hard
ARCHFLAGS += -mfpu=fpv5-sp-d16
ARCHFLAGS += -mthumb

################################################################
# CPPFLAGS
################################################################
CPPFLAGS =
CPPFLAGS += -MMD
CPPFLAGS += -MP

################################################################
# COMPILATION_FLAGS
################################################################
COMPILATION_FLAGS =
# COMPILATION_FLAGS += -Werror
COMPILATION_FLAGS += -Wall
COMPILATION_FLAGS += -fno-common
COMPILATION_FLAGS += -ffunction-sections
COMPILATION_FLAGS += -fdata-sections
COMPILATION_FLAGS += -ffreestanding
COMPILATION_FLAGS += -fno-builtin
COMPILATION_FLAGS += -fstack-usage
COMPILATION_FLAGS += -fno-exceptions
COMPILATION_FLAGS += -fshort-enums
# COMPILATION_FLAGS += -fPIE
ifeq ($(COMPILER),gcc)
COMPILATION_FLAGS += -mapcs-frame
endif

ifeq ($(cmse),s)
# enable trustzone
COMPILATION_FLAGS += -mcmse
else ifeq ($(cmse),ns)
# enable trustzone
COMPILATION_FLAGS += -mcmse
endif
COMPILATION_FLAGS += -std=gnu99

################################################################
# CFLAGS
################################################################
SDK_CONFIG_FLAGS = $(sdk_config:%=-D%)
INCLUDEDIR_FLAGS = $(includedirs:%=-I%)

CFLAGS =
CFLAGS += $(SDK_CONFIG_FLAGS)
CFLAGS += $(OPTIMIZTION_FLAGS)
CFLAGS += $(TARGET_FLAG)
CFLAGS += $(ARCHFLAGS)
CFLAGS += $(COMPILATION_FLAGS)
CFLAGS += $(INCLUDEDIR_FLAGS)
ifeq ($(COMPILER),clang)
CFLAGS += --sysroot $(LLVM_SYSROOT)
endif

################################################################
# ASFLAGS
################################################################
ASFLAGS =
ASFLAGS += -D__STARTUP_CLEAR_BSS
ASFLAGS += $(OPTIMIZTION_FLAGS)
ASFLAGS += $(TARGET_FLAG)
ASFLAGS += $(ARCHFLAGS)
ASFLAGS += $(COMPILATION_FLAGS)

################################################################
# LLFLAGS
################################################################
ifeq ($(COMPILER),clang)
LLFLAGS =
LLFLAGS += $(TARGET_FLAG)
LLFLAGS += $(ARCHFLAGS)
LLFLAGS += $(COMPILATION_FLAGS)
endif

################################################################
# LDFLAGS
################################################################
LIBDIR_FLAGS = $(libdirs:%=-L%)

LDFLAGS =
ifneq ($(findstring clang,$(LD)),)
LDFLAGS += $(TARGET_FLAG)
endif
LDFLAGS += $(ARCHFLAGS)
LDFLAGS += $(COMPILATION_FLAGS)

ifneq ($(findstring clang,$(LD)),)
LDFLAGS += -nostdlib
LDFLAGS += --sysroot $(LLVM_SYSROOT)
endif

ifneq ($(findstring gcc,$(LD)),)
# LDFLAGS += --specs=nano.specs
LDFLAGS += --specs=nosys.specs
endif

LDFLAGS += $(LIBDIR_FLAGS)
LDFLAGS += -Wl,-z,muldefs
LDFLAGS += -Wl,-z,noexecstack
LDFLAGS += -Wl,-static
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,-Map=$(targetdir)/$(project).map

ifneq ($(findstring gcc,$(LD)),)
LDFLAGS += -Wl,--print-memory-usage
endif

ifeq ($(cmse),s)
LDFLAGS += -Wl,--cmse-implib
LDFLAGS += -Wl,--out-implib=$(targetdir)/cmselib.o
else ifeq ($(cmse),ns)
endif

$(if $(linker_script),,$(error Linker script not specified!))
LDFLAGS += -T$(linker_script)

################################################################
# LIBFLAGS
################################################################
LIBFLAGS =
LIBFLAGS += -Wl,--start-group
LIBFLAGS += -lm
ifneq ($(findstring clang,$(LD)),)
LIBFLAGS += -lc
LIBFLAGS += $(LLVM_SYSROOT)/lib/crt0.o
LIBFLAGS += -lclang_rt.builtins-armv8m.main
LIBFLAGS += -lnosys
endif
ifneq ($(findstring gcc,$(LD)),)
endif
ifneq ($(lib),)
LIBFLAGS += $(lib)
endif
LIBFLAGS += -Wl,--end-group

################################################################
# all
################################################################
hex = $(targetdir)/$(project).hex
binary = $(targetdir)/$(project).bin
objdump = $(targetdir)/$(project).objdump.txt

ifeq ($(COMPILER),clang)
bc = $(targetdir)/$(project).bc
ir = $(targetdir)/$(project).ll
all: $(ir) $(bc) $(hex) $(binary) $(objdump) compile_commands.json
else ifeq ($(COMPILER),gcc)
all: $(hex) $(binary) $(objdump)
endif

################################################################
# src
################################################################
$(asm): Makefile config.mk
	@touch $@
$(src): Makefile config.mk
	@touch $@

################################################################
# assembly objects
################################################################
s_obj =
s_obj += $(asm:%=$(objdir)/%.o)

ifeq ($(COMPILER),clang)
$(s_obj): $(objdir)/%.o: %
	@mkdir -p $(@D)
	@mkdir -p $(dir $(depdir)/$*)
	@mkdir -p $(dir $(dbdir)/$*)
	$(info AS        $<)
	@$(AS) $(CPPFLAGS) -MF $(depdir)/$*.d -MJ $(dbdir)/$*.json -c $(ASFLAGS) -o $@ $<
	@$(OBJDUMP) -x -d -S --wide $@ > $@.objdump.txt
else ifeq ($(COMPILER),gcc)
$(s_obj): $(objdir)/%.o: %
	@mkdir -p $(@D)
	@mkdir -p $(dir $(depdir)/$*)
	@mkdir -p $(dir $(dbdir)/$*)
	$(info AS        $<)
	@$(AS) $(CPPFLAGS) -MF $(depdir)/$*.d -c $(ASFLAGS) -o $@ $<
	@$(OBJDUMP) -x -d -S --wide $@ > $@.objdump.txt
endif

################################################################
# ir
################################################################
ifeq ($(COMPILER),clang)
c_ll =
c_ll += $(src:%=$(irdir)/%.ll)

$(c_ll): $(irdir)/%.ll: %
	@mkdir -p $(@D)
	@mkdir -p $(dir $(depdir)/$*)
	@mkdir -p $(dir $(dbdir)/$*)
	$(info CC        $<)
	@$(CC) $(CPPFLAGS) -MF $(depdir)/$*.d -MJ $(dbdir)/$*.json -S -emit-llvm $(CFLAGS) -fno-discard-value-names -Xclang -disable-O0-optnone -o $@ $<
	@$(LLVMOPT) -S -passes=mem2reg $@ -o $@
endif

################################################################
# llvmlink ir/bc
################################################################
ifeq ($(COMPILER),clang)
$(ir): $(c_ll)
	@mkdir -p $(targetdir)
	$(info LLVMLINK  $@)
	@$(LLVMLINK) -S -o $@ $^

$(bc): $(c_ll)
	@mkdir -p $(targetdir)
	$(info LLVMLINK  $@)
	@$(LLVMLINK) -o $@ $^
endif

################################################################
# C objects
################################################################
c_obj =

ifeq ($(COMPILER),clang)
# c_obj += $(src:%=$(objdir)/%.o)

# $(c_obj): $(objdir)/%.o: %
# 	@mkdir -p $(@D)
# 	@mkdir -p $(dir $(depdir)/$*)
# 	@mkdir -p $(dir $(dbdir)/$*)
# 	$(info CC  $<)
# 	@$(CC) $(CPPFLAGS) -MF $(depdir)/$*.d -MJ $(dbdir)/$*.json $(CFLAGS) -c -o $@ $<

c_obj += $(c_ll:$(irdir)/%.ll=$(objdir)/%.o)

$(c_obj): $(objdir)/%.o: $(irdir)/%.ll
	@mkdir -p $(@D)
	$(info ASLL      $<)
	@$(CC) -c $(LLFLAGS) -o $@ $<
	@$(OBJDUMP) -x -d -S --wide $@ > $@.objdump.txt

else ifeq ($(COMPILER),gcc)
c_obj += $(src:%=$(objdir)/%.o)

$(c_obj): $(objdir)/%.o: %
	@mkdir -p $(@D)
	@mkdir -p $(dir $(depdir)/$*)
	$(info CC        $<)
	@$(CC) $(CPPFLAGS) -MF $(depdir)/$*.d -c $(CFLAGS) -o $@ $<
	@$(OBJDUMP) -x -d -S --wide $@ > $@.objdump.txt
endif

################################################################
# dep
################################################################
dep =
dep += $(src:%=$(depdir)/%.d)
dep += $(asm:%=$(depdir)/%.d)

$(dep): $(depdir)/%.d: $(objdir)/%.o

include $(dep)

################################################################
# compilation database
################################################################
ifeq ($(COMPILER),clang)
db =
db += $(src:%=$(dbdir)/%.json)
db += $(asm:%=$(dbdir)/%.json)

$(db): $(dbdir)/%.json: $(objdir)/%.o

compile_commands.json: $(db)
	$(info GEN       compile_commands.json)
	@sed -e '1s/^/[\n/' -e '$$ s/,$$/\n]/' $^ > $@
endif

################################################################
# elf
################################################################
elf = $(targetdir)/$(project).elf

obj =
obj += $(s_obj)
obj += $(c_obj)

$(elf): $(linker_script) $(obj)
	@mkdir -p $(targetdir)
	$(info LD        $@)
	@$(LD) $(LDFLAGS) -o $@ $(filter-out $<,$^) $(LIBFLAGS)

################################################################
# hex
################################################################
$(hex): $(elf)
	$(info GEN       $@)
	@$(OBJCOPY) -O ihex $^ $@
$(binary): $(elf)
	$(info GEN       $@)
	@$(OBJCOPY) -O binary $^ $@

################################################################
# objdump
################################################################
# $(objdump): $(elf)
# 	@$(OBJDUMP) -x -D -S -z --wide $^ > $@
$(objdump): $(elf)
	@$(OBJDUMP) -d -S -w $^ > $@

################################################################
# download
################################################################
script.jlink: script.jlink.template
	$(info GEN       script.jlink)
	@sed -e "s|%HEX_FILE_PATH%|$(shell pwd)/output/$(project).hex|" script.jlink.template > script.jlink

download: all script.jlink
	$(info Downloading...)
	@$(JLINK_EXE) -device "LPC55S69_M33_0" -if "SWD" -speed 4000 -autoconnect 1 -CommandFile "$(shell pwd)/script.jlink" > download.log

################################################################
# print
################################################################
print-macros:
	@$(CC) $(CFLAGS) -E -dM -x c /dev/null

################################################################
# clean
################################################################
clean:
	$(info Cleaning...)
	@rm -rf $(depdir)
	@rm -rf $(irdir)
	@rm -rf $(asmdir)
	@rm -rf $(objdir)
	@rm -rf $(dbdir)
	@rm -rf $(targetdir)
	@rm -rf script.jlink
	@find . -name "*.bak" -exec rm {} \;

.PHONY: all download clean
