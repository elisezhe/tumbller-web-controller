
.PHONY: format
format:
	find src include \( -name "*.cpp" -o -name "*.h" \) -print -exec clang-format -i {} \;
