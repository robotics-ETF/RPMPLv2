.PHONY: clean build run_tests

clean:
	rm -r ./build

build:
	mkdir -p build
	cmake -B build -DCMAKE_BUILD_TYPE=Release
	cmake --build build --config Release

run_tests:
	if [ -d build/tests ]; then ctest --test-dir build/tests; fi
