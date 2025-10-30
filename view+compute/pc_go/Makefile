

.PHONY: all
all: .out/pc_go

.out:
	mkdir -p .out

.out/pc_go: .out
	go build -o $@ .

.PHONY: run
run:
	go run .