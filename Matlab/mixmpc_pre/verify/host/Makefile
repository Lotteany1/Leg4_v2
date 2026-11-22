CC       ?= cc
CFLAGS   ?= -std=c99 -O2 -Wall -Wextra -Wshadow
SOLVER    = ../../deploy/solver
STAGING   = ../../build/staging
OUT       = ../../build/host
SHIM      = ../../Tool/solver
MATH      = ../../../../Wheel_Leg_Code/Code/Math
CPPFLAGS  = -I$(SOLVER) -I$(STAGING) -I$(OUT) -I$(SHIM) -I$(MATH) -I.
LDLIBS    = -lm

CORE  = $(SOLVER)/tinympc14.c
GEN   = $(STAGING)/tinympc14_data.c
# 输出层装配的表常量与两个原语来自 Tool 的部署副本，先跑 Tool 的 build_leg_tool 再编这里
TOOL  = $(MATH)/Leg_Comp.c $(MATH)/Leg_SpringF.c
PORT  = tinympc14_port_host.c
TEST  = tinympc14_solver_host_test.c
HDR   = $(wildcard $(SOLVER)/*.h $(STAGING)/*.h $(OUT)/*.h ./*.h)
BIN   = $(OUT)/tinympc14_solver_host_test

$(BIN): $(CORE) $(GEN) $(TOOL) $(PORT) $(TEST) $(HDR)
	mkdir -p $(OUT)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $(CORE) $(GEN) $(TOOL) $(PORT) $(TEST) $(LDLIBS)

run: $(BIN)
	$(BIN)

asan:
	mkdir -p $(OUT)
	$(CC) $(CPPFLAGS) -std=c99 -O1 -g -fsanitize=address,undefined \
		$(CORE) $(GEN) $(TOOL) $(PORT) $(TEST) -o $(BIN)_asan $(LDLIBS)
	$(BIN)_asan

clean:
	rm -f $(BIN) $(BIN)_asan $(BIN).exe

.PHONY: run asan clean