.PHONY: clean build run_tests

clean:
	rm -r ./build

build:
	mkdir -p build
	cd build && cmake .. && make

run_tests:
	ctest --test-dir build/tests -C Release --output-on-failure
