#include <iostream>

int main() {
	int x;
	std::cout << "Enter a first number : " << std::flush;
	std::cin >> x;

	int y;
	std::cout << "Enter a second number : " << std::flush;
	std::cin >> y;

	if (x >= y) {
		std::cout << x - y << std::endl;
	} else {
		std::cout << y - x << std::endl;
	}

	return 0;
}