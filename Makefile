CC     := gcc
LD      := $(CC)

INC     := -Isrc -Imjpro150/include
CFLAGS  := -fPIC -std=gnu11 -Wall -Wextra -Wpedantic -Ofast -march=native -flto
LDFLAGS := -shared -flto -L./mjpro150/bin -Wl,-rpath,'$$ORIGIN/mjpro150/bin'
LIBS    := -lmujoco150 -lglew -lGL -l:libglfw.so.3
OUT     := libslip.so

SRC     := $(wildcard src/*.c)
OBJ     := $(patsubst src/%.c,obj/%.o,$(SRC))

vpath %.c src

define make-goal
$1/%.o: %.c
	$(CC) $(CFLAGS) $(INC) -MMD -c $$< -o $$@
endef

all: checkdirs lib test

clean:
	rm -f $(OUT)
	rm -rf obj/
	rm -f test
	rm -f slipvis
	rm -f slip.h

test: lib
	$(CC) main.c $(INC) -L. -lslip -lm -Wl,-rpath,'$$ORIGIN',-rpath-link,'./mjpro150/bin' -o test

vis: lib
	$(CC) cassievis.c -Isrc -L. -lslip -Wl,-rpath,'$$ORIGIN',-rpath-link,'./mjpro150/bin' -o slipvis

lib: $(OBJ)
	$(LD) $^ -o $(OUT) $(LDFLAGS) $(LIBS)
	cp ./src/slip.h ./

checkdirs: obj

obj:
	@mkdir -p $@

$(foreach bdir,obj,$(eval $(call make-goal,$(bdir))))

.PHONY: all checkdirs clean

-include $(OBJ:%.o=%.d)
