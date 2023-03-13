#include <iostream>
#include <cmath>

int num_divisors(int n) {
	int count = 0;
	for (int i = 1; i <= std::sqrt(n); i++) {
		if (n % i == 0) count += 2;
		if (i * i == n) count--;
	}

	return count;
}

int main(void) {

	int inputNum = 0;

	std::cout << "Enter a positive integer: " << std::flush;

	std::cin >> inputNum;

	for (int i = 1; i <= inputNum; i++) {
		std::cout << i << '\t' << num_divisors(i) << std::endl;
	}
	
	return 0;
}
