.PHONY: deck all clean

deck:
	cargo zigbuild --release --target x86_64-unknown-linux-musl

all: deck
	@echo "cross compiling for steam deck"

clean:
	cargo clean
