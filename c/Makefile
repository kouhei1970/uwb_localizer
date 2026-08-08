# uwb_loc C 版
#
#   make            静的ライブラリ libuwbloc.a を作る
#   make test       テストを走らせる
#   make examples   サンプルを作る
#   make float      float 版で作り直して試す (マイコン想定)
#   make strict     -Wall -Wextra -Werror -pedantic で通るか見る
#   make size       セクションごとのサイズを出す
#   make clean

CC      ?= cc
AR      ?= ar
CFLAGS  ?= -O2 -g
CFLAGS  += -std=c99 -Iinclude -Isrc
WARN     = -Wall -Wextra -pedantic -Wshadow -Wconversion -Wno-sign-conversion
LDLIBS   = -lm

SRC  = src/uwb_linalg.c src/uwb_model.c src/uwb_closed_form.c src/uwb_nls.c src/uwb_ekf.c
OBJ  = $(SRC:.c=.o)
LIB  = libuwbloc.a

EXAMPLES = examples/01_snapshot examples/02_tracking examples/03_replay

.PHONY: all test examples float strict size clean

all: $(LIB)

$(LIB): $(OBJ)
	$(AR) rcs $@ $(OBJ)

%.o: %.c
	$(CC) $(CFLAGS) $(WARN) -c $< -o $@

test: tests/test_uwb
	./tests/test_uwb

tests/test_uwb: tests/test_uwb.c $(LIB)
	$(CC) $(CFLAGS) $(WARN) $< $(LIB) $(LDLIBS) -o $@

examples: $(EXAMPLES)

examples/%: examples/%.c $(LIB)
	$(CC) $(CFLAGS) $(WARN) $< $(LIB) $(LDLIBS) -o $@

# 単精度でも通るか。マイコンでは float の方が速い/小さいことが多い。
# -Wconversion を外すのは、テストのアンカー座標が double リテラルで
# 書いてあるため。ライブラリ本体は float でも無警告で通る。
float: clean
	$(MAKE) CFLAGS="-O2 -g -std=c99 -Iinclude -Isrc -DUWB_USE_FLOAT" \
	        WARN="-Wall -Wextra -pedantic -Wshadow" test

# 警告をすべてエラーにして通す。移植先のコンパイラで詰まらないように。
strict: clean
	$(MAKE) WARN="-Wall -Wextra -Werror -pedantic -Wshadow -Wconversion -Wno-sign-conversion" test

# ROM/RAM の見積もり。マイコンに載るかの目安。
size: $(LIB)
	@size -t $(OBJ) 2>/dev/null || echo "size コマンドが無い"

clean:
	rm -f $(OBJ) $(LIB) tests/test_uwb tests/*.o $(EXAMPLES)
