.PHONY: clean build run_tests

clean:
	rm -r ./build

build:
	mkdir -p build
	cd build && cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=1 && make 

run_tests:

